#include "serialport.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <sys/ioctl.h>

namespace Hlk {

std::thread *SerialPort::m_pollingThread = nullptr;
bool SerialPort::m_threadRunning = false;
std::vector<pollfd> SerialPort::m_pollFds;
std::vector<SerialPort *> SerialPort::m_instances;
std::mutex SerialPort::m_mutex;

SerialPort::SerialPort()
        : m_fd(0), 
          m_baudRate(BaudRate::BR_115200), 
          m_dataBits(DataBits::DB_8), 
          m_parity(Parity::NONE), 
          m_stopBits(StopBits::ONE) { 
    memset(&m_options, 0, sizeof(m_options));
}

SerialPort::~SerialPort() {
    close();
}

SerialPort::RetCode SerialPort::open(const std::string &port) {
    if (m_fd != 0) { // already opened
        return RetCode::ALREADY_OPENED;
    }

    m_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_fd == -1) { // error occurred
        m_fd = 0;
        switch (errno) {
        case ENOENT:
            return RetCode::DEVICE_NOT_CONNECTED;
        default:
            return RetCode::UNDEFINED_ERROR;
        }
    }

    // clear I/O buffers
    auto ret = ioctl(m_fd, TCFLSH, 2);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::BUFFER_FLUSH_ERROR;
    }

    // set descriptor to non-blocking mode
    ret = fcntl(m_fd, F_SETFL, FNDELAY);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::SWITCH_TO_NON_BLOCKING_MODE_ERROR;
    }

    // get current serial port options
    ret = tcgetattr(m_fd, &m_options);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::FAILED_TO_GET_PORT_OPTIONS;
    }

    // set local mode and enable receiver
    m_options.c_cflag |= (CLOCAL | CREAD);

    // to raw mode
    cfmakeraw(&m_options);

    // set speed (baud)
    switch (m_baudRate) {
    case BaudRate::BR_0:
        cfsetispeed(&m_options, B0);
        cfsetospeed(&m_options, B0);
        break;
        
    case BaudRate::BR_50:
        cfsetispeed(&m_options, B50);
        cfsetospeed(&m_options, B50);
        break;

    case BaudRate::BR_75:
        cfsetispeed(&m_options, B75);
        cfsetospeed(&m_options, B75);
        break;

    case BaudRate::BR_110:
        cfsetispeed(&m_options, B110);
        cfsetospeed(&m_options, B110);
        break;

    case BaudRate::BR_134:
        cfsetispeed(&m_options, B134);
        cfsetospeed(&m_options, B134);
        break;

    case BaudRate::BR_150:
        cfsetispeed(&m_options, B150);
        cfsetospeed(&m_options, B150);
        break;

    case BaudRate::BR_200:
        cfsetispeed(&m_options, B200);
        cfsetospeed(&m_options, B200);
        break;

    case BaudRate::BR_300:
        cfsetispeed(&m_options, B300);
        cfsetospeed(&m_options, B300);
        break;

    case BaudRate::BR_600:
        cfsetispeed(&m_options, B600);
        cfsetospeed(&m_options, B600);
        break;

    case BaudRate::BR_1200:
        cfsetispeed(&m_options, B1200);
        cfsetospeed(&m_options, B1200);
        break;

    case BaudRate::BR_1800:
        cfsetispeed(&m_options, B1800);
        cfsetospeed(&m_options, B1800);
        break;

    case BaudRate::BR_2400:
        cfsetispeed(&m_options, B2400);
        cfsetospeed(&m_options, B2400);
        break;

    case BaudRate::BR_4800:
        cfsetispeed(&m_options, B4800);
        cfsetospeed(&m_options, B4800);
        break;

    case BaudRate::BR_9600:
        cfsetispeed(&m_options, B9600);
        cfsetospeed(&m_options, B9600);
        break;

    case BaudRate::BR_19200:
        cfsetispeed(&m_options, B19200);
        cfsetospeed(&m_options, B19200);
        break;

    case BaudRate::BR_38400:
        cfsetispeed(&m_options, B38400);
        cfsetospeed(&m_options, B38400);
        break;

    case BaudRate::BR_57600:
        cfsetispeed(&m_options, B57600);
        cfsetospeed(&m_options, B57600);
        break;
    
    case BaudRate::BR_115200:
        cfsetispeed(&m_options, B115200);
        cfsetospeed(&m_options, B115200);
        break;
    } // switch (baudRate_)

    // set data bits
    // options_.c_cflag &= ~CSIZE; // wtf this doesn't support by kernel
    switch (m_dataBits) {    
    case DataBits::DB_5:
        m_options.c_cflag |= CS5;
        break;
    
    case DataBits::DB_6:
        m_options.c_cflag |= CS6;
        break;
    
    case DataBits::DB_7:        
        m_options.c_cflag |= CS7;
        break;
    
    case DataBits::DB_8:
        m_options.c_cflag |= CS8;
        break;
    } // switch (dataBits_)  

    // set parity
    switch (m_parity) {
    case Parity::NONE:
        m_options.c_cflag &= ~PARENB;
        break;

    case Parity::EVEN:
        m_options.c_cflag |= PARENB;
        m_options.c_cflag &= ~PARODD;
        break;

    case Parity::ODD:
        m_options.c_cflag |= PARENB;
        m_options.c_cflag |= PARODD;
        break;

    case Parity::SPACE:
        m_options.c_cflag &= ~PARENB;
        break;
    } // switch (parity_)

    // set stop bits
    switch(m_stopBits) {
    case StopBits::ONE:
        m_options.c_cflag &= ~CSTOPB;
        break;
    
    case StopBits::TWO:
        m_options.c_cflag |= CSTOPB;
        break;
    } // switch (stopBits_)

    // apply options
    ret = tcsetattr(m_fd, TCSANOW, &m_options);
    if (ret == -1) {
        ::close(m_fd);
        m_fd = 0;
        return RetCode::FAILED_TO_SET_PORT_OPTIONS;
    }

    // create polling fd and push it into static vector
    pollfd poll_fd;
    poll_fd.fd = m_fd;
    poll_fd.events = POLLIN;
    m_pollFds.push_back(poll_fd);

    // push current instance
    m_instances.push_back(this);

    // create static read thread if not exist
    if (!m_pollingThread) {
        m_threadRunning = true;
        m_pollingThread = new std::thread(polling);
    }

    return RetCode::SUCCESS;
}

void SerialPort::close() {
    // already closed
    if (m_fd == 0) {
        return;
    }

    // lock mutex to prevent "out of range" exception in polling
    m_mutex.lock();

    if (m_pollFds.size()) {
        // find closing descriptors and erase it from vectors
        for (size_t i = 0; i < m_pollFds.size(); ++i) {
            if (m_pollFds[i].fd == m_fd) {
                m_pollFds.erase(m_pollFds.begin() + i);
                m_instances.erase(m_instances.begin() + i);
            }
        }
    } else if (m_pollingThread) { // close polling thread
        m_mutex.unlock();
        m_threadRunning = false;
        m_pollingThread->join();
        delete m_pollingThread;
        m_pollingThread = nullptr;
    }

    // close serial port file descriptor
    ::close(m_fd);
    m_fd = 0;

    m_mutex.unlock();
}

SerialPort::RetCode SerialPort::write(const void *data, size_t length) {
    // write message
    ssize_t bytesWritten = ::write(m_fd, data, length);

    // check errors
    if (bytesWritten == -1) {
        switch (errno) {
        case EBADF:
            return RetCode::PORT_NOT_OPENED;
        default:
            return RetCode::UNDEFINED_ERROR;
        }
    }

    // check bytes written
    if (static_cast<size_t>(bytesWritten) < length) {
        return RetCode::NOT_ALL_WRITTEN;
    }

    return RetCode::SUCCESS;
}

void SerialPort::polling() {
    size_t bufSize = 1024;
    void *buf[bufSize];
    while (m_threadRunning) {
        int ret = poll(m_pollFds.data(), m_pollFds.size(), 3000);
        if (ret == -1) { // polling error occurred

        } else if (ret == 0) { // nothing happened

        } else { // process ports events
            m_mutex.lock();
            for (size_t i = 0; i < m_pollFds.size(); ++i) {
                if (m_pollFds[i].revents & POLLIN) {
                    m_pollFds[i].revents = 0;

                    ssize_t bytesRead = read(m_pollFds[i].fd, buf, bufSize);
                    if (bytesRead <= 0) {
                        m_mutex.unlock();
                        m_instances[i]->close();
                        switch (bytesRead) {
                        case 0:
                            m_instances[i]->onError(RetCode::DEVICE_REMOVED_DURING_OPERATION);
                            break;

                        case -1:
                            m_instances[i]->onError(RetCode::READ_ERROR);
                            break;
                        }
                        m_mutex.lock();
                        continue;
                    }
                    m_mutex.unlock();
                    m_instances[i]->onData(buf, bytesRead);
                    m_mutex.lock();
                }
            }
            m_mutex.unlock();
        }
    }
}

} // namespace Hlk