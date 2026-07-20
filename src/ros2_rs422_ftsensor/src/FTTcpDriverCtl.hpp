
#ifndef PROJECT_FTTCPDRIVERCTRL_H
#define PROJECT_FTTCPDRIVERCTRL_H

#include <QObject>
#include <QThread>
#include <QString>
#include <vector>
#include "FTTcpDriver.h"
#include <condition_variable>

class FTTcpDriverCtl : public QObject {
Q_OBJECT
protected:
    QThread workerThread;

public:
    FTTcpDriver *driver;

public:
    FTTcpDriverCtl() : QObject() {
        driver = new FTTcpDriver();
        driver->moveToThread(&workerThread);
        connect(&workerThread, &QThread::finished, driver, &QObject::deleteLater);
        connect(this, SIGNAL(start()), driver, SLOT(Start()));
        connect(this, SIGNAL(setoption(QString, int)), driver, SLOT(SetOption(QString, int)));
        connect(this, SIGNAL(writedata(char*, int)), driver, SLOT(WriteData(char*, int)));
        connect(this, SIGNAL(writedata()), driver, SLOT(WriteData()));
//        connect(this, SIGNAL(readdata(QByteArray)), driver, SLOT(ReadData(QByteArray)));
        connect(driver, SIGNAL(notifyFTData(QByteArray)), this, SLOT(onUpdateFTData(QByteArray)));
        connect(this, SIGNAL(stop()), driver, SLOT(Stop()));
        connect(driver, SIGNAL(statuschanged(int)), this, SLOT(onRealTimeHandState(int)));
        workerThread.start();
    }

    ~FTTcpDriverCtl() {
        workerThread.quit();
        workerThread.wait();
    }

    void SetOption(QString _IP, int _Port) {
        Q_EMIT setoption(_IP, _Port);
    }
    void Start() {
        Q_EMIT start();
    }
    void WriteData(std::vector<char> _Data) {
        buffer = _Data;
        Q_EMIT writedata(buffer.data(), buffer.size());
    }
    void WriteData() {
        Q_EMIT writedata();
    }
    void ReadData(std::vector<char> &_Data) {
        _Data.clear();
        for(int ii=0;ii<FTData.size();ii++)
            _Data.push_back(FTData.at(ii));
        FTData.clear();
    }
    int GetStatus() {
        return TcpState;
    }
    void Stop() {
        Q_EMIT stop();
    }

Q_SIGNALS:
    void setoption(QString _IP, int _Port);
    void start();
    void writedata(char* _Data, int _Len);
    void writedata();
    void readdata(QByteArray _Data);
    void getstatus(int &_status);
    void stop();

protected slots:
    void onRealTimeHandState(int _status) {
        TcpState = _status;
    }

    void onUpdateFTData(QByteArray _Data) {
        FTData = _Data;
    }

private:
    QByteArray FTData;
    std::vector<char> buffer;
    int TcpState = 0;
};

#endif //PROJECT_FTTCPDRIVERCTRL_H
