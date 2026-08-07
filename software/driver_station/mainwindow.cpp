#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->controls_config_tabWidget->setCurrentIndex(0);

    // Needed for the cross-thread (queued) signal/slot connections below.
    qRegisterMetaType<RobotDisplayData>("RobotDisplayData");
    qRegisterMetaType<QNetworkInterface>("QNetworkInterface");

    // Setup the network interface dropdown
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces())
    {
        for (const QNetworkAddressEntry &entry : iface.addressEntries())
        {
            if (entry.ip().protocol() != QAbstractSocket::IPv4Protocol) {continue;}

            QString text = QString("%1 : %2").arg(iface.humanReadableName()).arg(entry.ip().toString());
            ui->network_comboBox->addItem(text, iface.name());
        }
    }
    connect(ui->network_comboBox, &QComboBox::currentIndexChanged, this, &MainWindow::onInterfaceChanged);

    // Setup the version
    ui->ds_ver_Label->setText("V" + m_version);
    this->setWindowTitle("bellman Driver Station V" + m_version);
    QStringList ver_list = m_version.split(".");
    if (ver_list.size() == 3)
    {
        m_majorVersion = ver_list[0]; // Get the minor version from the overall version
        m_minorVersion = ver_list[1]; // Get the minor version from the overall version
    }

    // Setup the enable/disable buttons
    connect(ui->enable_pushButton, &QPushButton::clicked, this, &MainWindow::onEnableClicked);
    connect(ui->disable_pushButton, &QPushButton::clicked, this, &MainWindow::onDisableClicked);

    // Setup the restart robot code/controller buttons
    connect(ui->restart_code_pushButton, &QPushButton::clicked, this, &MainWindow::onRestartCodeClicked);
    connect(ui->restart_controller_pushButton, &QPushButton::clicked, this, &MainWindow::onRestartControllerClicked);

    // Setup the save configuration button
    connect(ui->save_pushButton, &QPushButton::clicked, this, &MainWindow::onSaveConfigClicked);

    // Setup the ID text box
    connect(ui->id_lineEdit, &QLineEdit::editingFinished, this, &MainWindow::onIDEdited);

    // Setup the debug messages checkbox
    connect(ui->debug_checkBox, &QCheckBox::toggled, this, &MainWindow::onDebugClicked);

    // Setup the move window button
    connect(ui->move_window_pushButton, &QPushButton::clicked, this, &MainWindow::moveToBottomCenter);

    // Setup the opmode combo box
    connect(ui->opmode_comboBox, &QComboBox::currentTextChanged, this, &MainWindow::onOpmodeChanged);

    // Input stuff
    SDL_Init(SDL_INIT_JOYSTICK);
    m_hatDirectionWidgets = {
                             { SDL_HAT_CENTERED, ui->direction_widget_0 },
                             { SDL_HAT_UP, ui->direction_widget_1 },
                             { SDL_HAT_RIGHT, ui->direction_widget_2 },
                             { SDL_HAT_DOWN, ui->direction_widget_3 },
                             { SDL_HAT_LEFT, ui->direction_widget_4 },
                             { SDL_HAT_RIGHTUP, ui->direction_widget_5 },
                             { SDL_HAT_RIGHTDOWN, ui->direction_widget_6 },
                             { SDL_HAT_LEFTUP, ui->direction_widget_7 },
                             { SDL_HAT_LEFTDOWN, ui->direction_widget_8 },
                             };

    // Setup controls list
    ui->controls_list->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->controls_list, &QListWidget::customContextMenuRequested, this, &MainWindow::onControlsContextMenuRequested);

    ui->controls_list->clear();
    refreshJoystickList();

    m_controlState = new ControlState();
    m_worker = new NetworkWorker(m_controlState); // no parent, about to be moved to another thread
    m_networkThread = new QThread(this);
    m_worker->moveToThread(m_networkThread);

    connect(m_networkThread, &QThread::started,  m_worker, &NetworkWorker::initialize);

    connect(this, &MainWindow::reconfigureNetwork, m_worker, &NetworkWorker::reconfigure);
    connect(m_worker, &NetworkWorker::robotDataReceived, this, &MainWindow::onRobotDataReceived);
    connect(m_worker, &NetworkWorker::communicationsLost, this, &MainWindow::onCommunicationsLost);
    connect(m_worker, &NetworkWorker::receiverCreated, this, &MainWindow::onReceiverCreated);

    m_networkThread->start(QThread::TimeCriticalPriority);

    // Load settings (AFTER everything is setup so it doesnt attempt to call into a nullptr)
    loadSavedSettings();

    // Start the GUI update loop
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateLoop);
    timer->start(5); // every 5ms
}

// Restores the robot ID and network interface from saved
void MainWindow::loadSavedSettings()
{
    int savedId = m_settings.savedRobotId();
    if (savedId >= 0)
    {
        ui->id_lineEdit->setText(QString::number(savedId));
        onIDEdited();
    }

    QString savedInterface = m_settings.savedInterface();
    if (!savedInterface.isEmpty())
    {
        int index = ui->network_comboBox->findData(savedInterface);
        if (index >= 0)
            ui->network_comboBox->setCurrentIndex(index); // triggers onInterfaceChanged
        else
            logMessage(ui->log_TextBrowser, "WARNING", "Driver Station", "Saved network interface is no longer available");
    }
}

void MainWindow::updateLoop()
{
    if (m_robotEnabled)
    {
        ui->status_Label->setText("ENABLED");
        ui->status_Label->setStyleSheet("color:green;");
        ui->opmode_comboBox->setDisabled(true);
    }
    else
    {
        ui->status_Label->setText("DISABLED");
        ui->status_Label->setStyleSheet("color:red;");
        ui->opmode_comboBox->setDisabled(false);
    }

    if (m_lastReceived != 0)
    {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        ui->age_Label->setText(QString::number((std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() - m_lastReceived) / 1000) + "ms");
    }
    else { ui->age_Label->setText("None"); }

    if (m_multicastIp != "") {ui->multicast_ip_Label->setText(m_multicastIp);}
    else {ui->multicast_ip_Label->setText("None");}

    if (m_robotIp != "") {ui->robot_ip_Label->setText(m_robotIp);}
    else {ui->robot_ip_Label->setText("Unknown");}

    publishJoystickSnapshot();

    // Check for joystick hotplugs
    SDL_Event event;
    bool changed = false;

    while (SDL_PollEvent(&event))
    {
        if (event.type == SDL_EVENT_JOYSTICK_ADDED || event.type == SDL_EVENT_JOYSTICK_REMOVED) { changed = true; }

        SDL_JoystickID *ids = SDL_GetJoysticks(&m_joystickCount);
        SDL_free(ids); // free immediately if you only need the count

        if (m_joystickCount == 0) {ui->controls_widget->setStyleSheet("background-color:red");}
        else {ui->controls_widget->setStyleSheet("background-color:green");}

        if ((m_joystickCount < m_startingJoystickCount) && m_controlState->enabledRequest())
        {
            disable();
            resetGUI();
            logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Joystick disconnected while enabled");
        }
    }

    if (changed) {refreshJoystickList(); }

    // Update the joystick UI (display only)
    QListWidgetItem *item = ui->controls_list->currentItem();
    if (!item) { return; }

    SDL_JoystickID id = item->data(Qt::UserRole).toUInt();
    SDL_Joystick *joy = m_joysticks.value(id);
    if (!joy) { return; }

    // Update the joysticks
    int numAxes = SDL_GetNumJoystickAxes(joy);
    numAxes = std::clamp(numAxes, 0, 10);
    for (int axis = 0; axis < numAxes; ++axis)
    {
        QProgressBar* bar = qobject_cast<QProgressBar*>(ui->axes_verticalLayout->itemAt(axis)->widget());
        Sint16 value = SDL_GetJoystickAxis(joy, axis);
        float normalized = (value + 32768.0f) / 65535.0f;
        int value_int = static_cast<int>(normalized * 100.0f + 0.5f);
        bar->setValue(value_int);
    }

    // Update the buttons
    int numButtons = SDL_GetNumJoystickButtons(joy);
    numButtons = std::clamp(numButtons, 0, 20);
    for (int button = 0; button < numButtons; ++button)
    {
        QWidget* indicator;
        if (button < 10) {indicator = ui->buttons_verticalLayout_1->itemAt(button)->widget();}
        else if (button >= 10) {indicator = ui->buttons_verticalLayout_2->itemAt(button-10)->widget();}
        bool pressed = SDL_GetJoystickButton(joy, button);
        if (pressed) {indicator->setStyleSheet("background-color:green");}
        else {indicator->setStyleSheet("background-color:red");}
    }

    // Update the direction
    for (QHash<Uint8, QWidget*>::const_iterator it = m_hatDirectionWidgets.constBegin(); it != m_hatDirectionWidgets.constEnd(); ++it)
    {
        static const QString red   = "background-color: red;";
        static const QString green = "background-color: green;";
        Uint8 state = SDL_GetJoystickHat(joy, 0);
        it.value()->setStyleSheet(it.key() == state ? green : red);
    }
}

void MainWindow::publishJoystickSnapshot()
{
    QHash<int, JoystickSnapshot> snapshot;

    for (int user_id = 0; user_id < ui->controls_list->count(); ++user_id)
    {
        SDL_JoystickID id = ui->controls_list->item(user_id)->data(Qt::UserRole).toUInt();
        SDL_Joystick *joy = m_joysticks.value(id);
        if (!joy)
            continue;

        JoystickSnapshot snap;

        int numAxes = SDL_GetNumJoystickAxes(joy);
        snap.axes.reserve(numAxes);
        for (int axis = 0; axis < numAxes; ++axis)
            snap.axes.append(SDL_GetJoystickAxis(joy, axis));

        int numButtons = SDL_GetNumJoystickButtons(joy);
        snap.buttons.reserve(numButtons);
        for (int button = 0; button < numButtons; ++button)
            snap.buttons.append(SDL_GetJoystickButton(joy, button));

        snap.hat = SDL_GetJoystickHat(joy, 0);

        snapshot.insert(user_id, snap);
    }

    m_controlState->setJoystickSnapshots(snapshot);
}

// Runs when the interface dropdown box is changed and (re)configures the receiver
void MainWindow::onInterfaceChanged(int index)
{
    if (index <= 0)
        return;

    m_interfaceId = index;
    m_controlState->setEnabledRequest(false);
    m_lastReceived = 0;
    resetGUI();

    QString ifaceName = ui->network_comboBox->currentData().toString();

    m_settings.persistInterfaceName(ifaceName);

    if (ifaceName.isEmpty())
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "No valid interface to create receiver on");
        return;
    }

    if (m_multicastIp == "")
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "No valid ID to create receiver on, please set a valid robot ID");
        return;
    }

    QNetworkInterface iface = QNetworkInterface::interfaceFromName(ifaceName);
    emit reconfigureNetwork(m_multicastIp, iface);
}

// Runs (on the GUI thread) whenever the network thread has processed a robot packet
void MainWindow::onRobotDataReceived(RobotDisplayData data)
{
    m_lastReceived = data.receivedAtUs;
    m_robotEnabled = data.enabled;
    m_robotEStopped = data.eStopped;

    if (data.forceReset) { resetGUI(); }

    if (data.updateOpmodes)
    {
        ui->opmode_comboBox->clear();
        ui->opmode_comboBox->addItems(QStringList(data.opmodes.cbegin(), data.opmodes.cend()));
    }

    for (const RobotLogEntry &entry : data.logs)
    {
        if (entry.level == "DEBUG" && !m_showDebug) { continue; }
        logMessage(ui->log_TextBrowser, entry.level, entry.caller, entry.contents);
    }

    ui->battery_progressBar->setValue(data.batteryPercent);
    ui->battery_progressBar->setFormat(QString::number(data.batteryVolts) + "V");
    setProgressBarColor(ui->battery_progressBar, colorFromPercent(data.batteryPercent));

    ui->cpu_progressBar->setValue((int)data.cpu);
    setProgressBarColor(ui->cpu_progressBar, colorFromPercent(100 - (int)data.cpu));

    ui->ram_progressBar->setValue((int)data.ram);
    setProgressBarColor(ui->ram_progressBar, colorFromPercent(100 - (int)data.ram));

    ui->communications_widget->setStyleSheet("background-color:green");

    ui->ver_label->setText(data.version);

    // Handle robot code/DS version mismatch
    if (!m_versionWarned)
    {
        QStringList robot_ver_list = data.version.split(".");
        if (robot_ver_list.size() == 3)
        {
            if (robot_ver_list[0] != m_majorVersion || robot_ver_list[1] != m_minorVersion)
            {
                m_controlState->requestEStop();
                logMessage(ui->log_TextBrowser, "FATAL", "Driver Station", "Robot/DS version mismatch, expected " + m_majorVersion + "." + m_minorVersion +", got " + robot_ver_list[0] + "." + robot_ver_list[1]);
                m_versionWarned = true;
            }
        }
    }

    m_robotIp = data.robotIp;
}

// Runs when the network thread detects the 50ms staleness timeout
void MainWindow::onCommunicationsLost()
{
    m_lastReceived = 0;
    logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Lost communication with robot");
    disable();
    resetGUI();
}

// Runs once the network thread has (re)bound the receiver for a new interface/ID
void MainWindow::onReceiverCreated(QString multicastIp)
{
    logMessage(ui->log_TextBrowser, "INFO", "Driver Station", "Created robot status receiver on UDP 11140");
}

// Runs when the enable button is clicked
void MainWindow::onEnableClicked()
{
    if (!m_controlState->commsReady())
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Unable to enable robot, no communications");
        return;
    }
    if (m_joystickCount == 0)
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Unable to enable robot, no controls");
        return;
    }
    if(ui->opmode_comboBox->currentText() == "Select an Opmode")
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Unable to enable robot, no opmode selected");
    }
    if(m_robotEStopped)
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Unable to enable robot, robot is E-Stopped");
    }

    m_startingJoystickCount = m_joystickCount;
    m_controlState->setEnabledRequest(true);
}

// Runs when the disable button is clicked
void MainWindow::onDisableClicked() { disable(); }

// Helper function to disable the robot and update the GUI
void MainWindow::disable()
{
    m_controlState->setEnabledRequest(false);
}

// Override function to detect enter/space presses
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space)
    {
        m_controlState->requestEStop();
        logMessage(ui->log_TextBrowser, "FATAL", "Driver Station", "Spacebar E-Stop");
    }

    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)
    {
        if (m_robotEnabled)
        {
            disable();
            logMessage(ui->log_TextBrowser, "WARNING", "Driver Station", "Return-key disable");
        }
        return;
    }

    QMainWindow::keyPressEvent(event);
}

// Runs when the Restart Robot Code button is clicked
void MainWindow::onRestartCodeClicked()
{
    if (!m_robotEStopped) {m_controlState->requestRestartCode();}
    else {logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Cannot restart robot code while emergency stopped");}
}

// Runs when the Restart Robot Controller button is clicked
void MainWindow::onRestartControllerClicked()
{
    if (!m_robotEStopped) {m_controlState->requestRestartController();}
    else {logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Cannot restart robot controller while emergency stopped");}
}

// Runs when the ID line edit is edited
void MainWindow::onIDEdited()
{
    bool ok = false;
    int id = ui->id_lineEdit->text().toInt(&ok);
    if (!ok || id > 25599 || id < 0)
    {
        logMessage(ui->log_TextBrowser, "ERROR", "Driver Station", "Please enter a valid robot ID below 25599");
        m_multicastIp = "";
        return;
    }

    m_multicastIp = "239." + QString::number(id / 100) + "." + QString::number(id % 100) + ".0"; // Calculate the multicast IP based on the robot ID
    onInterfaceChanged(m_interfaceId); // Recreate a sender/receiver for the new ID
    m_settings.persistRobotId(id);
    return;
}

// Runs when the "show debug messages" checkbox is clicked
void MainWindow::onDebugClicked(bool checked) { m_showDebug = checked; }

// Runs when the "Save Configuration" button is clicked, just to be sure
void MainWindow::onSaveConfigClicked()
{
    m_settings.sync();
    logMessage(ui->log_TextBrowser, "INFO", "Driver Station", "Configuration saved");
}

// Right-click handler for controls_list
void MainWindow::onControlsContextMenuRequested(const QPoint &pos)
{
    QListWidgetItem *item = ui->controls_list->itemAt(pos);
    if (!item)
        return;

    int row = ui->controls_list->row(item);
    bool isLive = item->data(Qt::UserRole).toUInt() != 0;
    QString pinnedName = item->data(PinnedNameRole).toString();

    if (isLive)
    {
        QString name = item->text();
        bool pin = pinnedName.isEmpty();

        if (pin)
        {
            m_settings.persistControlIndex(name, row);
            logMessage(ui->log_TextBrowser, "INFO", "Driver Station", QString("Pinned \"%1\" to slot %2").arg(name).arg(row));
        }
        else
        {
            m_settings.removeControlIndex(name);
            logMessage(ui->log_TextBrowser, "INFO", "Driver Station", QString("Unpinned \"%1\" from slot %2").arg(name).arg(row));
        }

        setSlotLive(item, static_cast<SDL_JoystickID>(item->data(Qt::UserRole).toUInt()), name, pin);
    }
    else if (!pinnedName.isEmpty())
    {
        m_settings.removeControlIndex(pinnedName);
        setSlotGhost(item, QString()); // back to a blank, available slot
        logMessage(ui->log_TextBrowser, "INFO", "Driver Station", QString("Forgot saved controller \"%1\"").arg(pinnedName));
    }
}

// Resets the GUI to how it is to start when the robot disconnects
void MainWindow::resetGUI()
{
    ui->battery_progressBar->setValue(100);
    ui->battery_progressBar->setStyleSheet("QProgressBar::chunk { background: red; }");
    ui->cpu_progressBar->setValue(100);
    ui->cpu_progressBar->setStyleSheet("QProgressBar::chunk { background: red; }");
    ui->ram_progressBar->setValue(100);
    ui->ram_progressBar->setStyleSheet("QProgressBar::chunk { background: red; }");
    ui->communications_widget->setStyleSheet("background-color:red");
    ui->robot_ip_Label->setText("Unknown");
    ui->ver_label->setText("Unknown");
    ui->opmode_comboBox->clear();
    ui->opmode_comboBox->addItem("Select an opmode");
    m_versionWarned = false;
}

void MainWindow::refreshJoystickList()
{
    // Get what joysticks are currently connected
    int count = 0;
    SDL_JoystickID *ids = SDL_GetJoysticks(&count);

    QList<SDL_JoystickID> currentIds;
    for (int i = 0; i < count; ++i)
    {
        currentIds.append(ids[i]);
    }
    SDL_free(ids);

    QSet<SDL_JoystickID> currentIdSet(currentIds.begin(), currentIds.end());

    // Handle joysticks that disappeared
    for (int row = 0; row < ui->controls_list->count(); ++row)
    {
        QListWidgetItem *item = ui->controls_list->item(row);
        SDL_JoystickID id = static_cast<SDL_JoystickID>(item->data(Qt::UserRole).toUInt());

        if (id == 0 || currentIdSet.contains(id)) {continue;}

        if (SDL_Joystick *joy = m_joysticks.take(id)) {SDL_CloseJoystick(joy);}

        setSlotGhost(item, item->data(PinnedNameRole).toString());
    }

    const QHash<QString, int> pins = m_settings.savedControls();

    // Bring in anything newly plugged in
    for (SDL_JoystickID id : currentIds)
    {
        if (m_joysticks.contains(id)) {continue;} // already tracked, leave its slot alone

        SDL_Joystick *joy = SDL_OpenJoystick(id);
        if (!joy) {continue;}

        QString name = QString::fromUtf8(SDL_GetJoystickName(joy));

        int row = rowForNewJoystick(name, pins);
        if (row < 0)
        {
            SDL_CloseJoystick(joy); // at capacity, ignore extras
            continue;
        }

        QListWidgetItem *item = ensureRow(row);
        bool pinned = pins.value(name, -1) == row;
        setSlotLive(item, id, name, pinned);

        m_joysticks.insert(id, joy);
    }

    // Make sure every saved binding is represented somewhere
    for (QHash<QString, int>::const_iterator it = pins.constBegin(); it != pins.constEnd(); ++it)
    {
        const QString &name = it.key();
        int row = it.value();
        if (row < 0 || row >= kMaxJoysticks) {continue;}

        bool represented = false;
        for (int r = 0; r < ui->controls_list->count(); ++r)
        {
            QListWidgetItem *existing = ui->controls_list->item(r);
            bool sameLive = existing->data(Qt::UserRole).toUInt() != 0 && existing->text() == name;
            bool sameGhost = existing->data(PinnedNameRole).toString() == name;
            if (sameLive || sameGhost)
            {
                represented = true;
                break;
            }
        }
        if (represented) {continue;}

        QListWidgetItem *item = ensureRow(row);
        if (isRowEmpty(row)) {setSlotGhost(item, name);}
    }
}

// Picks a valid row for new joysticks
int MainWindow::rowForNewJoystick(const QString &name, const QHash<QString, int> &pins)
{
    for (int row = 0; row < ui->controls_list->count(); ++row)
    {
        QListWidgetItem *item = ui->controls_list->item(row);
        if (item->data(Qt::UserRole).toUInt() == 0 && item->data(PinnedNameRole).toString() == name)
            return row;
    }

    if (pins.contains(name))
    {
        int row = pins.value(name);
        if (row >= 0 && row < kMaxJoysticks && isRowEmpty(row))
            return row;
    }

    for (int row = 0; row < kMaxJoysticks; ++row)
    {
        if (isRowEmpty(row))
            return row;
    }

    return -1;
}

bool MainWindow::isRowEmpty(int row) const
{
    if (row < 0 || row >= kMaxJoysticks)
        return false;
    if (row >= ui->controls_list->count())
        return true;
    QListWidgetItem *item = ui->controls_list->item(row);
    return item->data(Qt::UserRole).toUInt() == 0 && item->data(PinnedNameRole).toString().isEmpty();
}

QListWidgetItem *MainWindow::ensureRow(int row)
{
    while (ui->controls_list->count() <= row)
    {
        QListWidgetItem *filler = new QListWidgetItem();
        ui->controls_list->addItem(filler);
        setSlotGhost(filler, QString()); // normalizes to a blank slot
    }
    return ui->controls_list->item(row);
}

// Connected joystick
void MainWindow::setSlotLive(QListWidgetItem *item, SDL_JoystickID id, const QString &name, bool pinned)
{
    item->setText(name);
    item->setData(Qt::UserRole, static_cast<uint>(id));
    item->setData(PinnedNameRole, pinned ? name : QString());

    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDropEnabled;
    if (!pinned) {flags |= Qt::ItemIsDragEnabled;}
    item->setFlags(flags);

    QFont font = item->font();
    font.setUnderline(pinned);
    item->setFont(font);
    item->setForeground(QColor("white"));
}

// Disconnected "ghost" joystick
void MainWindow::setSlotGhost(QListWidgetItem *item, const QString &name)
{
    item->setText(name);
    item->setData(Qt::UserRole, 0u);
    item->setData(PinnedNameRole, name);

    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsDropEnabled | (name.isEmpty() ? Qt::ItemIsEnabled : Qt::NoItemFlags));

    QFont font = item->font();
    font.setUnderline(!name.isEmpty());
    item->setFont(font);
    item->setForeground(name.isEmpty() ? QColor("white") : QColor("darkgray"));
}

// Simple function to convert from a percent to a red-yellow-green gradient
QColor MainWindow::colorFromPercent(int percent)
{
    percent = std::clamp(percent, 0, 100);
    int hue = percent * 120 / 100;
    return QColor::fromHsv(hue, 255, 255);
}

// Sets the color of a progress bar from a QColor
void MainWindow::setProgressBarColor(QProgressBar *bar, const QColor &color)
{
    if (!bar) return;
    QString style = QString(R"(QProgressBar::chunk { background-color: %1; })").arg(color.name());
    bar->setStyleSheet(style);
}

// Moves the window to the bottom center of the screen
void MainWindow::moveToBottomCenter()
{
    QScreen *screen = this->screen();
    if (!screen) { screen = QApplication::primaryScreen(); }
    QRect available = screen->availableGeometry();

    int x = available.x() + (available.width() - width()) / 2;
    int y = available.bottom() - height();

    move(x, y);
}

// Runs when the opmode combo box is updated
void MainWindow::onOpmodeChanged() { m_controlState->setOpmode(ui->opmode_comboBox->currentText()); }

MainWindow::~MainWindow()
{
    QMetaObject::invokeMethod(m_worker, [this]()
                              {
                                  delete m_worker;
                                  m_worker = nullptr;
                              }, Qt::BlockingQueuedConnection);

    m_networkThread->quit();
    m_networkThread->wait();

    delete m_controlState;

    for (SDL_Joystick *joy : std::as_const(m_joysticks)) {SDL_CloseJoystick(joy);}
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);

    delete ui;
}

