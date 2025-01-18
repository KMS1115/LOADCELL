#include "loadcell.h"

LoadCell::LoadCell(char* serialPort, speed_t baudRate)
    : mSerialPort(serialPort)
    , mBaudRate(baudRate)
{
    this->initialize();
}

void LoadCell::ParseData(double (&LoadCell)[4], int &timeStamp)
{
    char buffer[128];
    ssize_t bytes_read = read(mSerialFd, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0)
    {
        buffer[bytes_read] = '\0';
        mData += buffer;

        size_t newline_pos = mData.find('\n'); 
        while (newline_pos != std::string::npos)
        {
            std::string packet = mData.substr(0, newline_pos);
            mData.erase(0, newline_pos + 1);

            mStreamData.str(packet);
            mStreamData.clear();

            if (mStreamData.peek() == '*')
            {
                mStreamData.ignore();
            }

            mTokens.clear();
            while (std::getline(mStreamData, mToken, ','))
            {
                mTokens.push_back(mToken);
            }

            if (mTokens.size() == 5)
            {
                for (int idx = 0; idx < 4; idx++)
                {
                    LoadCell[idx] = std::stod(mTokens[idx + 1]);
                }
                timeStamp = std::stoi(mTokens[0]);
            }
            newline_pos = mData.find('\n');
        }
    }
}


void LoadCell::initialize()
{
    mSerialFd = open(mSerialPort, O_RDWR | O_NOCTTY);
    if (mSerialFd == -1)
    {
        std::cerr << "LoadCell Error : Unable to open serial port." << std::endl;
    }
    tcgetattr(mSerialFd, &mSerialOptions);

    cfsetospeed(&mSerialOptions, mBaudRate);
    cfsetispeed(&mSerialOptions, mBaudRate);

    mSerialOptions.c_cflag &= ~PARENB;   // No parity
    mSerialOptions.c_cflag &= ~CSTOPB;   // 1 stop bit
    mSerialOptions.c_cflag &= ~CSIZE;
    mSerialOptions.c_cflag |= CS8;       // 8 data bits
    mSerialOptions.c_cflag &= ~CRTSCTS;  // No hardware flow control
    mSerialOptions.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines
    mSerialOptions.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    mSerialOptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical mode
    mSerialOptions.c_oflag &= ~OPOST;    // Raw output

    tcsetattr(mSerialFd, TCSANOW, &mSerialOptions);

    std::cout << "LoadCell is initialized" << std::endl;
}

int main() {
    char serialPort[] = "/dev/ttyACM0";
    speed_t baudRate = B115200;

    LoadCell loadCell(serialPort, baudRate);

    double loadCellValues[4] = {0};
    int timeStamp = 0;

    while (true) {
        loadCell.ParseData(loadCellValues, timeStamp);
        std::cout << "timestamp : " << timeStamp << std::endl;
        std::cout << "LoadCell[1] : " << loadCellValues[0] << std::endl;
        std::cout << "LoadCell[2] : " << loadCellValues[1] << std::endl;
        std::cout << "LoadCell[3] : " << loadCellValues[2] << std::endl;
        std::cout << "LoadCell[4] : " << loadCellValues[3] << std::endl;
    }
}