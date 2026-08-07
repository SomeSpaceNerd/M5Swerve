#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <algorithm>
#include <cmath>
#include <QMainWindow>
#include <QProgressBar>
#include <QCheckBox>
#include <QTimer>
#include <QThread>
#include <QKeyEvent>
#include <QApplication>
#include <QScreen>
#include <SDL3/SDL.h>
#include "receiver.h"
#include "sender.h"
#include "logger.h"
#include "networkworker.h"
#include "settingsmanager.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QListWidgetItem;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent *event) override;

signals:
    void reconfigureNetwork(QString multicastIp, QNetworkInterface iface);

private slots:
    void onInterfaceChanged(int index);

    void onEnableClicked();
    void onDisableClicked();

    void onRestartCodeClicked();
    void onRestartControllerClicked();

    void onIDEdited();

    void onRobotDataReceived(RobotDisplayData data);
    void onCommunicationsLost();
    void onReceiverCreated(QString multicastIp);

    void onDebugClicked(bool checked);

    void moveToBottomCenter();

    void onOpmodeChanged();

    void onSaveConfigClicked();
    void onControlsContextMenuRequested(const QPoint &pos);

private:
    Ui::MainWindow *ui;

    SettingsManager m_settings;

    // Custom item-data role used on controls_list entries to remember which persisted joystick name a slot belongs to
    static constexpr int PinnedNameRole = Qt::UserRole + 1;

    static constexpr int kMaxJoysticks = 7;

    const QString m_version = "0.2.0-M5Swerve";
    QString m_majorVersion = "";
    QString m_minorVersion = "";

    bool m_versionWarned = false;

    QColor colorFromPercent(int percent);
    void setProgressBarColor(QProgressBar *bar, const QColor &color);
    void updateLoop();
    void disable();
    void resetGUI();
    void refreshJoystickList();
    void publishJoystickSnapshot();
    void loadSavedSettings();

    // controls_list slot helpers
    int rowForNewJoystick(const QString &name, const QHash<QString, int> &pins);
    bool isRowEmpty(int row) const;
    QListWidgetItem *ensureRow(int row);
    void setSlotLive(QListWidgetItem *item, SDL_JoystickID id, const QString &name, bool pinned);
    void setSlotGhost(QListWidgetItem *item, const QString &name);

    bool m_showDebug = false;

    long long m_lastReceived = 0;

    int m_interfaceId = -1;

    QString m_robotIp = "";
    bool m_robotEnabled = false;

    bool m_robotEStopped = false;

    QHash<SDL_JoystickID, SDL_Joystick*> m_joysticks;
    int m_joystickCount = 0;
    int m_startingJoystickCount = 0;

    QHash<Uint8, QWidget*> m_hatDirectionWidgets;

    QString m_multicastIp;

    // Networking
    ControlState *m_controlState = nullptr;
    QThread *m_networkThread = nullptr;
    NetworkWorker *m_worker = nullptr;
};
#endif // MAINWINDOW_H

