#ifndef SENDER_H
#define SENDER_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QNetworkInterface>

#include "ds_to_robot.pb.h"
#include <pb_encode.h>

class Sender : public QObject
{
    Q_OBJECT

public:
    explicit Sender(quint16 localPort, const QHostAddress& destinationAddress, quint16 destinationPort, const QNetworkInterface& interface, QObject *parent = nullptr);

    bool isValid() const;

    bool sendMessage(const DSToRobot& msg);

private:
    QUdpSocket* socket;
    QHostAddress destinationAddress;
    quint16 destinationPort;
    bool valid = false;
};

#endif // SENDER_H

