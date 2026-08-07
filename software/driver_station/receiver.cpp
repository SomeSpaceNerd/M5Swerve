#include "receiver.h"
#include <QDebug>

Receiver::Receiver(QObject *parent)
    : QObject(parent)
    , socket(new QUdpSocket(this))
{
    connect(socket, &QUdpSocket::readyRead, this, &Receiver::processPendingDatagrams);
}

bool Receiver::start(const QString& multicastAddress, quint16 port, const QNetworkInterface& iface)
{
    if (!socket->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        qWarning() << socket->errorString();
        return false;
    }

    if (!socket->joinMulticastGroup(QHostAddress(multicastAddress), iface))
    {
        qWarning() << socket->errorString();
        return false;
    }

    return true;
}

void Receiver::processPendingDatagrams()
{
    while (socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(socket->pendingDatagramSize()));

        QHostAddress sender;
        quint16 senderPort = 0;
        if (socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort) < 0)
            continue;

        RobotToDS msg = RobotToDS_init_zero;
        pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<const pb_byte_t*>(datagram.constData()), datagram.size());

        if (pb_decode(&stream, RobotToDS_fields, &msg))
        {
            emit messageReceived(msg, sender);
        }
        else
        {
            qWarning() << "Failed to parse protobuf message";
        }
    }
}

