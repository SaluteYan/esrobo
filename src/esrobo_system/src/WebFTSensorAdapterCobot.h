
#ifndef PROJECT_WEBFTSENSORADAPTER_H
#define PROJECT_WEBFTSENSORADAPTER_H

#include <mutex>
#include <QObject>
#include <QString>
#include <memory>
#include <QTimer>
#include <functional>

class FTTcpDriverCtl;
class FTSerialDriver;

class ForceSensorStreamObserver {
public:
    ForceSensorStreamObserver() {};
    ~ForceSensorStreamObserver() {};
    virtual void onForceSensorDataStreamUpdate(std::vector<double> &_ft) = 0;
};

class WebFTSensorAdapterCobot : public QObject {
	Q_OBJECT
public:
	WebFTSensorAdapterCobot(bool _longData = false);
	virtual ~WebFTSensorAdapterCobot();

	virtual bool start();
	virtual void stop();
	virtual void zero(bool flag){};
	virtual void attach(const std::shared_ptr<ForceSensorStreamObserver>& observer);
	virtual void setIp( const std::string &ip );
	virtual void setPort( const std::string &port );
    virtual int getStatus();
    virtual bool getftdata(std::vector<double > &_ft);

private:

	std::shared_ptr<FTTcpDriverCtl> _tcpdriver;
    std::shared_ptr<FTSerialDriver> _serialdriver;
	std::shared_ptr<QTimer> _timer;
	void sendString(QString message);
	QString request(QString message);

	QByteArray _serialBuffer;

	std::mutex _mutex;

	bool _longFormat;

	bool _isConnected;
	bool _isSerial = true;

	QString _ip;
	QString _port = "10000";
	std::vector<std::shared_ptr<ForceSensorStreamObserver> > _observers;

	std::vector<double> _wrench;

	void readRead();

	void onConnect();

	void onDisconnect();

	void notify(std::function<void(std::shared_ptr<ForceSensorStreamObserver>& observer)> func);

    int ConvData(int _Data);

private slots:
	void ReadFTData();
};

#endif //PROJECT_WEBFTSENSORADAPTER_H
