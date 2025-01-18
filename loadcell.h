//
// Created by pc on 1/7/25.
//

#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iomanip>
#include <vector>

class LoadCell {
  public:
    LoadCell(char* serialPort, speed_t baudRate);
    void ParseData(double (&LoadCell)[4],
                   int& timeStamp);

  private :
    void initialize();

  private :
    int mSerialFd;

    const char* mSerialPort;
    const speed_t mBaudRate;

    struct termios mSerialOptions;

    std::string mData;
    std::string mToken;
    std::vector<std::string> mTokens;
    std::stringstream mStreamData;
};


#endif //MAIN_H
