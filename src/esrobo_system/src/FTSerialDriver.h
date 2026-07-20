//
// Created by mp on 19-1-21.
//

#ifndef FTSERIAL_DRIVER_H
#define FTSERIAL_DRIVER_H
#include <stdio.h>
#include <strings.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/file.h>

class FTSerialDriver{
public:
	enum StopBits { Stop1_0, Stop1_5, Stop2_0 };
    enum Parity { Even, Odd, None, Mark, Space};
    enum DataBits {Data5, Data6, Data7, Data8};

public:
    FTSerialDriver();
    ~FTSerialDriver();
	bool OpenPort(const std::string &_port, int _baudrate, DataBits _dbits = Data8, Parity _parity = None, StopBits _stopbits = Stop1_0);
    int ReadPort(void *buf, int iByte);
    bool WritePort(const void *buf, int iByte);
    int ClosePort();
	int GetComListFromReg(std::vector<std::string> &_PortList);

public:
	bool PortState;
	std::string PortName;

private:
	int SetPort(int iBaudRate, int iDataSize = 8, char cParity = 'N', int iStopBit = 1);
    int fd;
};
  

  
#endif
