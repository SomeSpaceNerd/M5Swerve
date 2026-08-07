#ifndef RECEIVER_H
#define RECEIVER_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QNetworkInterface>

#include "pb_decode.h"
#include "robot_to_ds.pb.h"

class Receiver : public QObject
{
    Q_OBJECT

public:
    explicit Receiver(QObject *parent = nullptr);

    bool start(const QString& multicastAddress, quint16 port, const QNetworkInterface& iface);

signals:
    void messageReceived(const RobotToDS& message, const QHostAddress& sender);

private slots:
    void processPendingDatagrams();

private:
    QUdpSocket* socket;
};

#endif // RECEIVER_H

