#ifndef NETWORKWORKER_H
#define NETWORKWORKER_H

#include <atomic>
#include <QObject>
#include <QByteArray>
#include <QHash>
#include <QVector>
#include <QString>
#include <QMutex>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdio>
#include <cstring>
#include <QDebug>

#include "pb_decode.h"
#include "ds_to_robot.pb.h"
#include "robot_advertisement.pb.h"
#include "robot_log.pb.h"

#include "receiver.h"
#include "sender.h"

// Raw state of one joystick, published by the GUI thread (the only thread
// allowed to touch SDL) and consumed by the network thread when it builds an
// outgoing packet. Kept as raw axis/button values - the normalization math
// stays in NetworkWorker, unchanged from the original code, so the actual
// "networking behavior" doesn't move, only where it's read from.
struct JoystickSnapshot
{
    QVector<Sint16> axes;
    QVector<bool> buttons;
    Uint8 hat = SDL_HAT_CENTERED;
};

// A single line destined for the log browser. Kept as plain data so the
// network thread never touches a QTextBrowser (or any QWidget) directly.
struct RobotLogEntry
{
    QString level;
    QString caller;
    QString contents;
};

// Everything the GUI needs to refresh itself after one robot packet has been
// processed. Built on the network thread, consumed on the GUI thread via a
// queued signal.
struct RobotDisplayData
{
    bool enabled = false;
    bool eStopped = false;
    int batteryPercent = 0;
    float batteryVolts = 0.0f;
    float cpu = 0.0f;
    float ram = 0.0f;
    QString version;
    QString robotIp;
    long long receivedAtUs = 0;
    bool forceReset = false;
    QVector<RobotLogEntry> logs;
    QVector<QString> opmodes;
    bool updateOpmodes = false;
};

Q_DECLARE_METATYPE(RobotDisplayData)

class ControlState
{
public:
    void setJoystickSnapshots(const QHash<int, JoystickSnapshot> &snapshots);
    QHash<int, JoystickSnapshot> joystickSnapshots() const;

    void setEnabledRequest(bool enabled);
    bool enabledRequest() const;

    void requestRestartCode();
    bool consumeRestartCode();

    void requestRestartController();
    bool consumeRestartController();

    void requestEStop();
    bool consumeEStop();

    void setCommsReady(bool ready);
    bool commsReady() const;

    void setOpmode(QString opmode);
    QString consumeOpmode() const;

private:
    mutable QMutex m_joystickMutex;
    QHash<int, JoystickSnapshot> m_joystickSnapshots;

    std::atomic<bool> m_enabledRequest{false};
    std::atomic<bool> m_restartCode{false};
    std::atomic<bool> m_restartController{false};
    std::atomic<bool> m_eStopRequest{false};
    std::atomic<bool> m_commsReady{false};

    mutable QMutex m_opmodeMutex;
    QString m_opmode = "";
};

class NetworkWorker : public QObject
{
    Q_OBJECT

public:
    explicit NetworkWorker(ControlState *state, QObject *parent = nullptr);

public slots:
    void initialize();

    // (Re)creates the receiver/sender for a new interface or robot ID.
    void reconfigure(QString multicastIp, QNetworkInterface iface);

signals:
    void robotDataReceived(RobotDisplayData data);
    void communicationsLost();
    void receiverCreated(QString multicastIp);

private slots:
    void handleDatagram(const RobotToDS &msg, const QHostAddress &robotIp);
    void handleAdDatagram();
    void handleLogConnection();
    void handleLogReadyRead();
    void handleLogDisconnected();
    void sendControlPacket();
    void checkTimeout();

private:
    enum class LogDecodeResult
    {
        NeedMore,
        Ok,
        Invalid
    };

    static bool readVarint(const QByteArray &data, quint64 &value, int &bytesUsed);
    static LogDecodeResult decodeDelimitedRobotLog(const QByteArray &data, RobotLog &msg, int &bytesConsumed);

    void publishLogEntries(const QVector<RobotLogEntry> &entries);
    void resetConnectionState();
    void ensureRobotSender(const QHostAddress &robotIp);
    void closeRobotSender();
    void closeLogSocket();
    void closeAdSocket();

    ControlState *m_state;

    Receiver *m_receiver = nullptr;
    Sender *m_sender = nullptr;
    QNetworkInterface m_iface;

    QTimer *m_timeoutTimer = nullptr;
    QTimer *m_controlTimer = nullptr;

    QUdpSocket *m_adSocket = nullptr;
    QTcpServer *m_logServer = nullptr;
    QTcpSocket *m_logSocket = nullptr;
    QByteArray m_logBuffer;

    QHostAddress m_robotAddress;
    bool m_haveRobotAddress = false;

    long long m_lastStatusUs = 0;
    long long m_lastAdUs = 0;
    bool m_statusEstablished = false;
    quint32 m_prevReceiveSequence = 0;
    quint32 m_sequence = 0;
    bool m_updateOpmodes = true;
    RobotDisplayData m_cachedDisplay;
};

#endif // NETWORKWORKER_H

