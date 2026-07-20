//
// Created by mp on 19-1-29.
//

#ifndef NICENET_FTTCPDRIVER_H
#define NICENET_FTTCPDRIVER_H

#include <QTcpSocket>
#include <QTcpServer>
#include <QObject>
#include <memory>


class QTcpServer;
class QTcpSocket;

class FTTcpDriver : public QObject {
Q_OBJECT
public:
    FTTcpDriver();
    ~FTTcpDriver();
public slots:
    void SetOption(QString _IP, int _Port);
    void Start();
    void WriteData(char* _Data, int _Len);
    void WriteData();
    void ReadData(QByteArray _Data);
    int GetStatus();
    void Stop();

private:
    void onReadData();
    void onSocketConnect();
    void onSocketDisconnect();
    void onSocketError();

    signals:
    void statuschanged(int _status);
    void notifyFTData(QByteArray);

private:
    QTcpSocket *YSSocket;
    QString Ysip;
    int YsPort;
    QByteArray RecvData;
    int TcpState = 0;

};


#endif //NICENET_FTTCPDRIVER_H
