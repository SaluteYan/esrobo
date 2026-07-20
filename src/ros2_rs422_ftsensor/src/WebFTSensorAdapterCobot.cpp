
#include "WebFTSensorAdapterCobot.h"
#include "FTTcpDriverCtl.hpp"
#include "FTSerialDriver.h"
#include <iostream>

#define FFAC   (double)(40.0/4096.0)
#define TFAC   (double)(1.6/4096.0)

WebFTSensorAdapterCobot::WebFTSensorAdapterCobot(bool _longData) :
	QObject(nullptr),
    _timer(std::make_shared<QTimer>(this)),
	_isConnected(false),
    _longFormat(_longData)
{
//    connect(_timer.get(),&QTimer::timeout,this,&WebFTSensorAdapterCobot::readRead);
//    _timer->setInterval(10);
    QTimer *t1 = new QTimer(this);
    connect(t1, SIGNAL(timeout()), this, SLOT(ReadFTData()));
    t1->setInterval(5);
    t1->start();
    _tcpdriver = std::make_shared<FTTcpDriverCtl>();
    _serialdriver = std::make_shared<FTSerialDriver>();
//    _timer->start();
}

WebFTSensorAdapterCobot::~WebFTSensorAdapterCobot() {

}

void WebFTSensorAdapterCobot::ReadFTData(){

    if(_tcpdriver->GetStatus() || _serialdriver->PortState) {

        std::vector<char> _cmd;
        char _buff[512];
        bzero(_buff, 512);
        char *_pBuff = nullptr;
        int _bufLen = 0;
//        _tcpdriver->WriteData();
        if(_isSerial) {
//            usleep(5000);
            _bufLen = _serialdriver->ReadPort(_buff, 512);
            _pBuff = _buff;
//            _bufLen = 2400;
        } else {
            _tcpdriver->ReadData(_cmd);
            if(_cmd.size() >= 12) {
                _pBuff = _cmd.data();
                _bufLen = _cmd.size();
            }
        }

        std::lock_guard<std::mutex> lock_guard(_mutex);
        if (_pBuff) {
            _isConnected = true;
            if(!_longFormat) {
                for(int ii=0;ii<_bufLen;ii++) {

                    if ((unsigned char)_pBuff[ii] != 0x49 && (unsigned char)_pBuff[ii] != 0x48) continue;
                    if ((unsigned char)_pBuff[ii + 10] != 0x0d || (unsigned char)_pBuff[ii + 11] != 0x0a) return;
                    double f1 = ConvData((_pBuff[ii + 1] & 0xff) * 16 + (_pBuff[ii + 2] >> 4 & 0x0f)) * FFAC;
                    double f2 = ConvData((_pBuff[ii + 2] & 0x0f) * 256 + (_pBuff[ii + 3] & 0xff)) * FFAC;
                    double f3 = ConvData((_pBuff[ii + 4] & 0xff) * 16 + (_pBuff[ii + 5] >> 4 & 0x0f)) * FFAC;
                    double t1 = ConvData((_pBuff[ii + 5] & 0x0f) * 256 + (_pBuff[ii + 6] & 0xff)) * TFAC;
                    double t2 = ConvData((_pBuff[ii + 7] & 0xff) * 16 + (_pBuff[ii + 8] >> 4 & 0x0f)) * TFAC;
                    double t3 = ConvData((_pBuff[ii + 8] & 0x0f) * 256 + (_pBuff[ii + 9] & 0xff)) * TFAC;
                    int tmp = _pBuff[ii + 1];
                    std::cout  << f1 << " " <<  f2 << " " << f3 << " " << t1 << " " << t2 << " " << t3 << std::endl;
                    _wrench = {f1, f2, f3, t1, t2, t3};
                    notify([=](std::shared_ptr<ForceSensorStreamObserver> &observer) {
                        observer->onForceSensorDataStreamUpdate(_wrench);
                    });
                    break;
                } 
            } else {
                for(int ii=0;ii<_bufLen;ii++) {
                	//std::cout  << "data2 length: " << _bufLen << std::endl;
                    if ((unsigned char)_pBuff[ii] != 0x49 && (unsigned char)_pBuff[ii] != 0x48) continue;
                    //std::cout  << "data3 length: " << _bufLen << ", " << std::to_string((unsigned char)_pBuff[ii + 1])  << std::endl;
                    if ((unsigned char)_pBuff[ii + 1] != 0xaa) continue;
                    if ((unsigned char)_pBuff[ii + 26] != 0x0d || (unsigned char)_pBuff[ii + 27] != 0x0a) return;
                    
                    float f1, f2, f3, t1, t2, t3;
                    uchar _dataBufferF1[4] = {((unsigned char)_pBuff[ii + 2] & 0xff), ((unsigned char)_pBuff[ii + 3] & 0xff), ((unsigned char)_pBuff[ii + 4] & 0xff), ((unsigned char)_pBuff[ii + 5] & 0xff)};
                    memcpy((void *)&f1, _dataBufferF1, 4);

                    uchar _dataBufferF2[4] = {((unsigned char)_pBuff[ii + 6] & 0xff), ((unsigned char)_pBuff[ii + 7] & 0xff), ((unsigned char)_pBuff[ii + 8] & 0xff), ((unsigned char)_pBuff[ii + 9] & 0xff)};
                    memcpy((void *)&f2, _dataBufferF2, 4);

                    uchar _dataBufferF3[4] = {((unsigned char)_pBuff[ii + 10] & 0xff), ((unsigned char)_pBuff[ii + 11] & 0xff), ((unsigned char)_pBuff[ii + 12] & 0xff), ((unsigned char)_pBuff[ii + 13] & 0xff)};
                    memcpy((void *)&f3, _dataBufferF3, 4);

                    uchar _dataBufferT1[4] = {((unsigned char)_pBuff[ii + 14] & 0xff), ((unsigned char)_pBuff[ii + 15] & 0xff), ((unsigned char)_pBuff[ii + 16] & 0xff), ((unsigned char)_pBuff[ii + 17] & 0xff)};
                    memcpy((void *)&t1, _dataBufferT1, 4);

                    uchar _dataBufferT2[4] = {((unsigned char)_pBuff[ii + 18] & 0xff), ((unsigned char)_pBuff[ii + 19] & 0xff), ((unsigned char)_pBuff[ii + 20] & 0xff), ((unsigned char)_pBuff[ii + 21] & 0xff)};
                    memcpy((void *)&t2, _dataBufferT2, 4);

                    uchar _dataBufferT3[4] = {((unsigned char)_pBuff[ii + 22] & 0xff), ((unsigned char)_pBuff[ii + 23] & 0xff), ((unsigned char)_pBuff[ii + 24] & 0xff), ((unsigned char)_pBuff[ii + 25] & 0xff)};
                    memcpy((void *)&t3, _dataBufferT3, 4);

                    //std::cout << "ft data: "  << f1 << " " <<  f2 << " " << f3 << " " << t1 << " " << t2 << " " << t3 << std::endl;
                    _wrench = {f1, f2, f3, t1, t2, t3};
                    notify([=](std::shared_ptr<ForceSensorStreamObserver> &observer) {
                        observer->onForceSensorDataStreamUpdate(_wrench);
                    });
                    break;
                } 
            }
        }
//        _tcpdriver->WriteData();//        _tcpdriver->WriteData();

    } else {
        _isConnected = false;
    }
}

bool WebFTSensorAdapterCobot::start() {

    if(_isSerial) {
        std::string _PortName = "echo \"123\" | sudo -S chmod 777 "+_port.toStdString();
        system(_PortName.c_str());
        _serialdriver->OpenPort(_port.toStdString(), 460800);
        return true;
    }

    if(_tcpdriver && _tcpdriver) {
        std::cout << "ft: " << _ip.toStdString() << std::endl;
        _tcpdriver->SetOption(_ip, _port.toInt());
        _tcpdriver->Start();
//        _timer->start();
    }

	return true;
}

void WebFTSensorAdapterCobot::stop() {
    _tcpdriver->stop();
    _serialdriver->ClosePort();
//    notify([=](std::shared_ptr<ForceSensorStreamObserver>& observer) {
//        observer->onForceSensorDisconnect();
//    });
}

void WebFTSensorAdapterCobot::sendString(QString message){

}

QString WebFTSensorAdapterCobot::request(QString message){
//    sendString(message);
//    if (_serial->waitForReadyRead(30000)) {
//
//        // read request
//        QByteArray requestData = _serial->readAll();
//        return QString::fromUtf8(requestData);
//    }
    return "";
}

void WebFTSensorAdapterCobot::attach(const std::shared_ptr<ForceSensorStreamObserver>& observer) {
    std::lock_guard<std::mutex> lock_guard(_mutex);
	for (auto& iter : _observers) {
		if (iter.get() == observer.get()) {
			return; // Already have attached
		}
	}
	if (observer) {
		_observers.push_back(observer);
	}
}


void WebFTSensorAdapterCobot::notify(std::function<void(std::shared_ptr<ForceSensorStreamObserver>& observer)> func) {
	if (func) {
		std::vector<std::shared_ptr<ForceSensorStreamObserver> > observer_tmp;
		observer_tmp = _observers;
		for (auto& observer : observer_tmp) {
			func(observer);
		}
	}
}

void WebFTSensorAdapterCobot::setIp(const std::string &ip) {
    _ip = ip.c_str();
    _isSerial = false;
}

void WebFTSensorAdapterCobot::setPort(const std::string &port) {

    _port = port.c_str();

}

int WebFTSensorAdapterCobot::ConvData(int _Data) {
    if(_Data & 0x800) {
        return -(~(_Data - 1) & 0x7ff);
    } else {
        return (_Data & 0x7ff);
    }
}

int WebFTSensorAdapterCobot::getStatus() {
    return _isConnected;
}

bool WebFTSensorAdapterCobot::getftdata(std::vector<double> &_ft) {
    if(_isConnected) {
        _ft = _wrench;
        return true;
    } else {
        _ft.clear();
        return false;
    }
}
