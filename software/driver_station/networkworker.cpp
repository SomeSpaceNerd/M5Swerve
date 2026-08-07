#include "networkworker.h"

namespace {
long long nowUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}

void ControlState::setJoystickSnapshots(const QHash<int, JoystickSnapshot> &snapshots)
{
    QMutexLocker lock(&m_joystickMutex);
    m_joystickSnapshots = snapshots;
}

QHash<int, JoystickSnapshot> ControlState::joystickSnapshots() const
{
    QMutexLocker lock(&m_joystickMutex);
    return m_joystickSnapshots;
}

void ControlState::setEnabledRequest(bool enabled) { m_enabledRequest.store(enabled); }
bool ControlState::enabledRequest() const { return m_enabledRequest.load(); }

void ControlState::requestRestartCode() { m_restartCode.store(true); }
bool ControlState::consumeRestartCode() { return m_restartCode.exchange(false); }

void ControlState::requestRestartController() { m_restartController.store(true); }
bool ControlState::consumeRestartController() { return m_restartController.exchange(false); }

void ControlState::requestEStop() { m_eStopRequest.store(true); }
bool ControlState::consumeEStop() { return m_eStopRequest.exchange(false); }

void ControlState::setCommsReady(bool ready) { m_commsReady.store(ready); }
bool ControlState::commsReady() const { return m_commsReady.load(); }

// QString isn't trivially copyable, so it can't live in a std::atomic like the flags above - guard it with a mutex instead
void ControlState::setOpmode(QString opmode)
{
    QMutexLocker lock(&m_opmodeMutex);
    m_opmode = opmode;
}

QString ControlState::consumeOpmode() const
{
    QMutexLocker lock(&m_opmodeMutex);
    return m_opmode;
}

NetworkWorker::NetworkWorker(ControlState *state, QObject *parent) : QObject(parent), m_state(state)
{
}

void NetworkWorker::initialize()
{
    m_timeoutTimer = new QTimer(this);
    connect(m_timeoutTimer, &QTimer::timeout, this, &NetworkWorker::checkTimeout);
    m_timeoutTimer->start(5);

    m_controlTimer = new QTimer(this);
    connect(m_controlTimer, &QTimer::timeout, this, &NetworkWorker::sendControlPacket);
    m_controlTimer->start(10);
}

void NetworkWorker::closeRobotSender()
{
    if (m_sender)
    {
        m_sender->deleteLater();
        m_sender = nullptr;
    }
}

void NetworkWorker::closeLogSocket()
{
    if (m_logSocket)
    {
        m_logSocket->disconnect(this);
        m_logSocket->deleteLater();
        m_logSocket = nullptr;
    }
    m_logBuffer.clear();
}

void NetworkWorker::closeAdSocket()
{
    if (m_adSocket)
    {
        m_adSocket->disconnect(this);
        m_adSocket->deleteLater();
        m_adSocket = nullptr;
    }
}

void NetworkWorker::resetConnectionState()
{
    m_state->setEnabledRequest(false);
    m_state->setCommsReady(false);
    closeRobotSender();
    closeLogSocket();
    m_robotAddress = {};
    m_haveRobotAddress = false;
    m_lastStatusUs = 0;
    m_lastAdUs = 0;
    m_statusEstablished = false;
    m_prevReceiveSequence = 0;
    m_sequence = 0;
    m_updateOpmodes = true;
    m_cachedDisplay = RobotDisplayData{};
}

void NetworkWorker::reconfigure(QString multicastIp, QNetworkInterface iface)
{
    m_state->setCommsReady(false);

    if (m_receiver) { m_receiver->deleteLater(); m_receiver = nullptr; }
    closeRobotSender();
    closeLogSocket();
    closeAdSocket();

    m_iface = iface;
    resetConnectionState();

    m_receiver = new Receiver(this);
    connect(m_receiver, &Receiver::messageReceived, this, &NetworkWorker::handleDatagram);
    m_receiver->start(multicastIp, 11140, m_iface);

    m_adSocket = new QUdpSocket(this);
    connect(m_adSocket, &QUdpSocket::readyRead, this, &NetworkWorker::handleAdDatagram);

    if (!m_adSocket->bind(QHostAddress::AnyIPv4, 11150, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        qWarning() << "Failed to bind ad socket:" << m_adSocket->errorString();
        closeAdSocket();
    }
    else if (!m_adSocket->joinMulticastGroup(QHostAddress(multicastIp), m_iface))
    {
        qWarning() << "Failed to join ad multicast group:" << m_adSocket->errorString();
        closeAdSocket();
    }

    if (!m_logServer)
    {
        m_logServer = new QTcpServer(this);
        connect(m_logServer, &QTcpServer::newConnection, this, &NetworkWorker::handleLogConnection);
    }

    if (!m_logServer->isListening() && !m_logServer->listen(QHostAddress::AnyIPv4, 11140))
    {
        qWarning() << "Failed to listen for robot logs:" << m_logServer->errorString();
    }

    emit receiverCreated(multicastIp);
}

void NetworkWorker::ensureRobotSender(const QHostAddress &robotIp)
{
    if (robotIp.isNull())
        return;

    if (m_sender && m_haveRobotAddress && m_robotAddress == robotIp && m_sender->isValid())
        return;

    closeRobotSender();

    m_robotAddress = robotIp;
    m_haveRobotAddress = true;
    m_sender = new Sender(11130, robotIp, 11130, m_iface, this);

    if (!m_sender->isValid())
    {
        closeRobotSender();
        m_haveRobotAddress = false;
        return;
    }

    m_state->setCommsReady(true);
}

void NetworkWorker::handleAdDatagram()
{
    if (!m_adSocket)
        return;

    while (m_adSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(m_adSocket->pendingDatagramSize()));

        QHostAddress sender;
        quint16 senderPort = 0;
        if (m_adSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort) < 0)
            continue;

        RobotAd ad = RobotAd_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<const pb_byte_t*>(datagram.constData()), datagram.size());
        if (!pb_decode(&stream, RobotAd_fields, &ad))
            continue;

        m_lastAdUs = nowUs();

        // Ignore other robots once one is active.
        if (m_haveRobotAddress && sender != m_robotAddress)
            continue;

        if (ad.has_ds)
        {
            // Another DS already owns this robot.
            if (!m_sender)
            {
                m_state->setCommsReady(false);
            }
            continue;
        }

        ensureRobotSender(sender);
    }
}

void NetworkWorker::handleLogConnection()
{
    if (!m_logServer)
        return;

    while (m_logServer->hasPendingConnections())
    {
        QTcpSocket *socket = m_logServer->nextPendingConnection();
        if (!socket)
            continue;

        if (!m_haveRobotAddress || socket->peerAddress() != m_robotAddress)
        {
            socket->disconnectFromHost();
            socket->deleteLater();
            continue;
        }

        if (m_logSocket)
            closeLogSocket();

        m_logSocket = socket;
        m_logBuffer.clear();

        connect(m_logSocket, &QTcpSocket::readyRead, this, &NetworkWorker::handleLogReadyRead);
        connect(m_logSocket, &QTcpSocket::disconnected, this, &NetworkWorker::handleLogDisconnected);
    }
}

bool NetworkWorker::readVarint(const QByteArray &data, quint64 &value, int &bytesUsed)
{
    value = 0;
    bytesUsed = 0;

    for (int i = 0; i < data.size() && i < 10; ++i)
    {
        const quint8 byte = static_cast<quint8>(data[i]);
        value |= quint64(byte & 0x7F) << (7 * i);
        ++bytesUsed;
        if ((byte & 0x80) == 0)
            return true;
    }

    bytesUsed = 0;
    return false;
}

NetworkWorker::LogDecodeResult NetworkWorker::decodeDelimitedRobotLog(const QByteArray &data, RobotLog &msg, int &bytesConsumed)
{
    bytesConsumed = 0;

    quint64 messageLen = 0;
    int prefixLen = 0;
    if (!readVarint(data, messageLen, prefixLen))
    {
        // A varint is at most 10 bytes; if we already have that many buffered and still
        // haven't found a terminator, the stream is corrupt and will never resolve - don't
        // wait for bytes that aren't coming (bytesConsumed stays 0, caller must give up)
        return data.size() >= 10 ? LogDecodeResult::Invalid : LogDecodeResult::NeedMore;
    }

    if (messageLen > static_cast<quint64>(INT_MAX))
        return LogDecodeResult::Invalid;

    const int totalNeeded = prefixLen + static_cast<int>(messageLen);
    if (data.size() < totalNeeded)
        return LogDecodeResult::NeedMore;

    QByteArray payload = data.mid(prefixLen, static_cast<int>(messageLen));
    pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<const pb_byte_t*>(payload.constData()), payload.size());
    if (!pb_decode(&stream, RobotLog_fields, &msg))
    {
        bytesConsumed = totalNeeded;
        return LogDecodeResult::Invalid;
    }

    bytesConsumed = totalNeeded;
    return LogDecodeResult::Ok;
}

void NetworkWorker::publishLogEntries(const QVector<RobotLogEntry> &entries)
{
    if (entries.isEmpty())
        return;

    RobotDisplayData display = m_cachedDisplay;
    display.logs = entries;
    display.receivedAtUs = m_lastStatusUs != 0 ? m_lastStatusUs : nowUs();
    if (display.robotIp.isEmpty() && m_haveRobotAddress)
        display.robotIp = m_robotAddress.toString();
    emit robotDataReceived(display);
}

void NetworkWorker::handleLogReadyRead()
{
    if (!m_logSocket)
        return;

    m_logBuffer += m_logSocket->readAll();

    while (!m_logBuffer.isEmpty())
    {
        RobotLog logMsg = RobotLog_init_zero;
        int bytesConsumed = 0;
        LogDecodeResult result = decodeDelimitedRobotLog(m_logBuffer, logMsg, bytesConsumed);

        if (result == LogDecodeResult::NeedMore)
            return;

        if (bytesConsumed > 0)
            m_logBuffer.remove(0, bytesConsumed);

        if (result != LogDecodeResult::Ok)
        {
            // Corrupt framing with nothing we can safely skip means the stream can never
            // resync on its own - drop the connection instead of spinning forever on it
            if (bytesConsumed == 0)
            {
                qWarning() << "Corrupt robot log stream, disconnecting";
                closeLogSocket();
                return;
            }

            continue;
        }

        QVector<RobotLogEntry> entries;
        entries.reserve(logMsg.logs_count);

        for (pb_size_t i = 0; i < logMsg.logs_count; ++i)
        {
            const RobotLog_LogEntry &entry = logMsg.logs[i];
            entries.append({QString::fromUtf8(entry.level), QString::fromUtf8(entry.caller), QString::fromUtf8(entry.contents)});
        }

        publishLogEntries(entries);
    }
}

void NetworkWorker::handleLogDisconnected()
{
    closeLogSocket();
}

void NetworkWorker::sendControlPacket()
{
    if (!m_sender || !m_sender->isValid())
        return;

    if (!m_haveRobotAddress)
        return;

    DSToRobot outgoing = DSToRobot_init_zero;

    outgoing.enabled = m_state->enabledRequest();
    outgoing.restart_code = m_state->consumeRestartCode();
    outgoing.restart_controller = m_state->consumeRestartController();
    outgoing.e_stop = m_state->consumeEStop();

    std::snprintf(outgoing.opmode, sizeof(outgoing.opmode), "%s", m_state->consumeOpmode().toUtf8().constData());

    outgoing.sequence = m_sequence++;

    const QHash<int, JoystickSnapshot> snapshots = m_state->joystickSnapshots();
    for (QHash<int, JoystickSnapshot>::const_iterator it = snapshots.constBegin(); it != snapshots.constEnd(); ++it)
    {
        if (outgoing.controls_count >= sizeof(outgoing.controls) / sizeof(outgoing.controls[0]))
            break;

        DSToRobot_Joystick &joyEntry = outgoing.controls[outgoing.controls_count++];
        joyEntry = DSToRobot_Joystick_init_zero;
        joyEntry.id = it.key();

        const JoystickSnapshot &snap = it.value();

        for (int axis = 0; axis < snap.axes.size(); ++axis)
        {
            if (joyEntry.axes_count >= sizeof(joyEntry.axes) / sizeof(joyEntry.axes[0]))
                break;

            float normalized = snap.axes[axis] / 32767.0f;
            normalized = std::clamp(normalized, -1.0f, 1.0f);
            normalized = std::round(normalized * 100.0f) / 100.0f;
            joyEntry.axes[joyEntry.axes_count++] = normalized;
        }

        for (int button = 0; button < snap.buttons.size(); ++button)
        {
            if (joyEntry.buttons_count >= sizeof(joyEntry.buttons) / sizeof(joyEntry.buttons[0]))
                break;

            joyEntry.buttons[joyEntry.buttons_count++] = snap.buttons[button];
        }

        joyEntry.direction = snap.hat;
    }

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    outgoing.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    m_sender->sendMessage(outgoing);
}

void NetworkWorker::handleDatagram(const RobotToDS &msg, const QHostAddress &robotIp)
{
    if (m_haveRobotAddress && robotIp != m_robotAddress)
        return;

    if (!m_haveRobotAddress)
        ensureRobotSender(robotIp);

    m_lastStatusUs = nowUs();
    m_statusEstablished = true;

    if (msg.sequence == 0)
    {
        m_prevReceiveSequence = 0;
        m_updateOpmodes = true;
    }

    RobotDisplayData display;
    display.receivedAtUs = m_lastStatusUs;
    display.enabled = msg.enabled;
    display.eStopped = msg.e_stopped;
    display.batteryPercent = msg.battery_percent;
    display.batteryVolts = msg.battery_volts;
    display.cpu = msg.cpu;
    display.ram = msg.ram;
    display.version = QString::fromUtf8(msg.version);
    display.robotIp = robotIp.toString();
    display.updateOpmodes = m_updateOpmodes;

    if (m_updateOpmodes)
    {
        m_updateOpmodes = false;
    }

    display.opmodes.reserve(msg.opmodes_count);
    for (pb_size_t i = 0; i < msg.opmodes_count; ++i)
    {
        display.opmodes.append(QString::fromUtf8(msg.opmodes[i]));
    }

    if (msg.clear_enable)
        m_state->setEnabledRequest(false);

    if ((msg.battery_percent > 150) && msg.enabled)
    {
        m_state->setEnabledRequest(false);
        display.forceReset = true;
        display.logs.append({"ERROR", "Driver Station", "Battery percentage above range (check your HAL device properties)"});
    }

    m_cachedDisplay = display;
    m_cachedDisplay.logs.clear();

    // Widen to a signed 64-bit delta so a robot restart (sequence resetting below m_prevReceiveSequence)
    // cannot underflow into a bogus, huge unsigned "dropped packets" count
    const qint64 sequenceDelta = static_cast<qint64>(msg.sequence) - static_cast<qint64>(m_prevReceiveSequence);
    if (sequenceDelta > 5 && m_prevReceiveSequence != 0)
    {
        display.logs.append({"WARNING", "Driver Station", QString("Dropped %1 packets").arg(sequenceDelta - 1)});
    }
    m_prevReceiveSequence = msg.sequence;

    emit robotDataReceived(display);
}

void NetworkWorker::checkTimeout()
{
    const long long now = nowUs();

    if (!m_haveRobotAddress || !m_sender)
        return;

    const long long timeoutUs = m_statusEstablished ? 200000 : 500000;
    const long long lastActivity = m_statusEstablished ? m_lastStatusUs : m_lastAdUs;

    if (lastActivity != 0 && (now - lastActivity) > timeoutUs)
    {
        m_state->setEnabledRequest(false);
        m_state->setCommsReady(false);
        resetConnectionState();
        emit communicationsLost();
    }
}

