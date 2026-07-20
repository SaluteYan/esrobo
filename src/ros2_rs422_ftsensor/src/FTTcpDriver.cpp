//
// Created by mp on 19-1-29.
//

#include "FTTcpDriver.h"
#include <iostream>

FTTcpDriver::FTTcpDriver() {

    YSSocket = new QTcpSocket(this);
    YSSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    Ysip = "127.0.0.0";
    YsPort = 0;

    connect(YSSocket, &QTcpSocket::readyRead, this, &FTTcpDriver::onReadData);
    connect(YSSocket, &QTcpSocket::connected, this, &FTTcpDriver::onSocketConnect);
    connect(YSSocket, &QTcpSocket::disconnected, this, &FTTcpDriver::onSocketDisconnect);
    connect(YSSocket, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, &FTTcpDriver::onSocketError);
}

FTTcpDriver::~FTTcpDriver() {
    Stop();
    delete YSSocket;
}

void FTTcpDriver::SetOption(QString _IP, int _Port) {
    Ysip = _IP;
    YsPort = _Port;
}

void FTTcpDriver::Start() {

    if(YsPort == 0) return;
    if (!YSSocket->isOpen()) {
        std::cout << "connect ftsensor param: " << Ysip.toStdString() << ", " << YsPort << std::endl;
        YSSocket->connectToHost(Ysip, YsPort);
    }
}

int FTTcpDriver::GetStatus() {
    return TcpState;
}

void FTTcpDriver::Stop() {
    if (YSSocket->isOpen()) {
        YSSocket->close();
    }
}

void FTTcpDriver::WriteData(char* _Data, int _Len) {
    if (YSSocket->isOpen()) {
//        YSSocket->write(_Data, _Len);
    }
}

void FTTcpDriver::WriteData() {
    if (YSSocket->isOpen()) {
        char _ReadCMD[3] = {0x49, 0x0d, 0x0a};
//        QString _msg("R");
        YSSocket->write(_ReadCMD, 3);
    }
}

void FTTcpDriver::ReadData(QByteArray _Data) {
    if (YSSocket->isOpen()) {
        _Data = RecvData;
//        for(int ii=0;ii<RecvData.size();ii++)
//            _Data.push_back(RecvData.at(ii));
    }
}


void FTTcpDriver::onReadData() {
    if (YSSocket->isOpen()) {
        RecvData.clear();
        RecvData = YSSocket->readAll();
        emit notifyFTData(RecvData);
//        std::cout << "ft data: ";
//        for(int ii=0;ii<RecvData.size();ii++)
//            printf("%d ", RecvData.at(ii));
//        printf("\n");
    }
}

void FTTcpDriver::onSocketConnect() {
    std::cout << Ysip.toStdString() << ": ftsensor connected." << std::endl;
    TcpState = 1;
//    WriteData();
    Q_EMIT statuschanged(TcpState);
}

void FTTcpDriver::onSocketDisconnect() {
    std::cout << Ysip.toStdString() << ": ftsensor disconnected." << std::endl;
    TcpState = 0;
    Q_EMIT statuschanged(TcpState);
}

void FTTcpDriver::onSocketError() {
    std::cout << Ysip.toStdString() << ": ftsensor error." << std::endl;
    TcpState = 0;
    Q_EMIT statuschanged(TcpState);
}