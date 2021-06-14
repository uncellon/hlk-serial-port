#include "serialport.h"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <sys/ioctl.h>

namespace Hlk {

std::thread *SerialPort::pollingThread_ = nullptr;
bool SerialPort::threadRunning_ = false;
std::vector<pollfd> SerialPort::pollFds_;
std::vector<SerialPort *> SerialPort::instances_;
std::mutex SerialPort::mutex_;

SerialPort::SerialPort()
        : fd_(0), 
          baudRate_(BaudRate::BR_115200), 
          dataBits_(DataBits::DB_8), 
          parity_(Parity::NONE), 
          stopBits_(StopBits::ONE) { 
    memset(&options_, 0, sizeof(options_));
}

SerialPort::~SerialPort() {
    close();
}

SerialPort::RetCode SerialPort::open(const std::string &port) {
    if (fd_ != 0) { // already opened
        return RetCode::ALREADY_OPENED;
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd_ == -1) { // error occurred
        fd_ = 0;
        switch (errno) {
        case ENOENT:
            return RetCode::DEVICE_NOT_CONNECTED;
        default:
            return RetCode::UNDEFINED_ERROR;
        }
    }

    // clear I/O buffers
    auto ret = ioctl(fd_, TCFLSH, 2);
    if (ret == -1) {
        ::close(fd_);
        fd_ = 0;
        return RetCode::BUFFER_FLUSH_ERROR;
    }

    // set descriptor to non-blocking mode
    ret = fcntl(fd_, F_SETFL, FNDELAY);
    if (ret == -1) {
        ::close(fd_);
        fd_ = 0;
        return RetCode::SWITCH_TO_NON_BLOCKING_MODE_ERROR;
    }

    // get current serial port options
    ret = tcgetattr(fd_, &options_);
    if (ret == -1) {
        ::close(fd_);
        fd_ = 0;
        return RetCode::FAILED_TO_GET_PORT_OPTIONS;
    }

    // set local mode and enable receiver
    options_.c_cflag |= (CLOCAL | CREAD);

    // to raw mode
    cfmakeraw(&options_);

    // set speed (baud)
    switch (baudRate_) {
    case BaudRate::BR_0:
        cfsetispeed(&options_, B0);
        cfsetospeed(&options_, B0);
        break;
        
    case BaudRate::BR_50:
        cfsetispeed(&options_, B50);
        cfsetospeed(&options_, B50);
        break;

    case BaudRate::BR_75:
        cfsetispeed(&options_, B75);
        cfsetospeed(&options_, B75);
        break;

    case BaudRate::BR_110:
        cfsetispeed(&options_, B110);
        cfsetospeed(&options_, B110);
        break;

    case BaudRate::BR_134:
        cfsetispeed(&options_, B134);
        cfsetospeed(&options_, B134);
        break;

    case BaudRate::BR_150:
        cfsetispeed(&options_, B150);
        cfsetospeed(&options_, B150);
        break;

    case BaudRate::BR_200:
        cfsetispeed(&options_, B200);
        cfsetospeed(&options_, B200);
        break;

    case BaudRate::BR_300:
        cfsetispeed(&options_, B300);
        cfsetospeed(&options_, B300);
        break;

    case BaudRate::BR_600:
        cfsetispeed(&options_, B600);
        cfsetospeed(&options_, B600);
        break;

    case BaudRate::BR_1200:
        cfsetispeed(&options_, B1200);
        cfsetospeed(&options_, B1200);
        break;

    case BaudRate::BR_1800:
        cfsetispeed(&options_, B1800);
        cfsetospeed(&options_, B1800);
        break;

    case BaudRate::BR_2400:
        cfsetispeed(&options_, B2400);
        cfsetospeed(&options_, B2400);
        break;

    case BaudRate::BR_4800:
        cfsetispeed(&options_, B4800);
        cfsetospeed(&options_, B4800);
        break;

    case BaudRate::BR_9600:
        cfsetispeed(&options_, B9600);
        cfsetospeed(&options_, B9600);
        break;

    case BaudRate::BR_19200:
        cfsetispeed(&options_, B19200);
        cfsetospeed(&options_, B19200);
        break;

    case BaudRate::BR_38400:
        cfsetispeed(&options_, B38400);
        cfsetospeed(&options_, B38400);
        break;

    case BaudRate::BR_57600:
        cfsetispeed(&options_, B57600);
        cfsetospeed(&options_, B57600);
        break;
    
    case BaudRate::BR_115200:
        cfsetispeed(&options_, B115200);
        cfsetospeed(&options_, B115200);
        break;
    } // switch (baudRate_)

    // set data bits
    // options_.c_cflag &= ~CSIZE; // wtf this doesn't support by kernel
    switch (dataBits_) {    
    case DataBits::DB_5:
        options_.c_cflag |= CS5;
        break;
    
    case DataBits::DB_6:
        options_.c_cflag |= CS6;
        break;
    
    case DataBits::DB_7:        
        options_.c_cflag |= CS7;
        break;
    
    case DataBits::DB_8:
        options_.c_cflag |= CS8;
        break;
    } // switch (dataBits_)  

    // set parity
    switch (parity_) {
    case Parity::NONE:
        options_.c_cflag &= ~PARENB;
        break;

    case Parity::EVEN:
        options_.c_cflag |= PARENB;
        options_.c_cflag &= ~PARODD;
        break;

    case Parity::ODD:
        options_.c_cflag |= PARENB;
        options_.c_cflag |= PARODD;
        break;

    case Parity::SPACE:
        options_.c_cflag &= ~PARENB;
        break;
    } // switch (parity_)

    // set stop bits
    switch(stopBits_) {
    case StopBits::ONE:
        options_.c_cflag &= ~CSTOPB;
        break;
    
    case StopBits::TWO:
        options_.c_cflag |= CSTOPB;
        break;
    } // switch (stopBits_)

    // apply options
    ret = tcsetattr(fd_, TCSANOW, &options_);
    if (ret == -1) {
        ::close(fd_);
        fd_ = 0;
        return RetCode::FAILED_TO_SET_PORT_OPTIONS;
    }

    // create polling fd and push it into static vector
    pollfd poll_fd;
    poll_fd.fd = fd_;
    poll_fd.events = POLLIN;
    pollFds_.push_back(poll_fd);

    // push current instance
    instances_.push_back(this);

    // create static read thread if not exist
    if (!pollingThread_) {
        threadRunning_ = true;
        pollingThread_ = new std::thread(polling);
    }

    return RetCode::SUCCESS;
}

void SerialPort::close() {
    // already closed
    if (fd_ == 0) {
        return;
    }

    // lock mutex to prevent "out of range" exception in polling
    mutex_.lock();

    if (pollFds_.size()) {
        // find closing descriptors and erase it from vectors
        for (size_t i = 0; i < pollFds_.size(); ++i) {
            if (pollFds_[i].fd == fd_) {
                pollFds_.erase(pollFds_.begin() + i);
                instances_.erase(instances_.begin() + i);
            }
        }
    } else if (pollingThread_) { // close polling thread
        mutex_.unlock();
        threadRunning_ = false;
        pollingThread_->join();
        delete pollingThread_;
        pollingThread_ = nullptr;
    }

    // close serial port file descriptor
    ::close(fd_);
    fd_ = 0;

    mutex_.unlock();
}

SerialPort::RetCode SerialPort::write(const void *data, size_t length) {
    // write message
    ssize_t bytesWritten = ::write(fd_, data, length);

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
    while (threadRunning_) {
        int ret = poll(pollFds_.data(), pollFds_.size(), 3000);
        if (ret == -1) { // polling error occurred

        } else if (ret == 0) { // nothing happened

        } else { // process ports events
            mutex_.lock();
            for (size_t i = 0; i < pollFds_.size(); ++i) {
                if (pollFds_[i].revents & POLLIN) {
                    pollFds_[i].revents = 0;

                    ssize_t bytesRead = read(pollFds_[i].fd, buf, bufSize);
                    if (bytesRead <= 0) {
                        mutex_.unlock();
                        instances_[i]->close();
                        switch (bytesRead) {
                        case 0:
                            instances_[i]->onError(RetCode::DEVICE_REMOVED_DURING_OPERATION);
                            break;

                        case -1:
                            instances_[i]->onError(RetCode::READ_ERROR);
                            break;
                        }
                        mutex_.lock();
                        continue;
                    }
                    mutex_.unlock();
                    instances_[i]->onData(buf, bytesRead);
                    mutex_.lock();
                }
            }
            mutex_.unlock();
        }
    }
}

} // namespace Hlk