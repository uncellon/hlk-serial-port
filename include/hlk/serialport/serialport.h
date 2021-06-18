#ifndef HLK_SERIAL_PORT
#define HLK_SERIAL_PORT

#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <termios.h>
#include <sys/poll.h>
#include <hlk/events/event.h>

namespace Hlk {

class SerialPort {
public:
    enum class BaudRate;
    enum class DataBits;
    enum class Parity;
    enum class StopBits;
    enum class RetCode;

    /***************************************************************************
     * Constructors / Destructors
     **************************************************************************/

    SerialPort();
    ~SerialPort();

    /***************************************************************************
     * Public methods
     **************************************************************************/

    /**
     * @brief Try to open serial port device
     * 
     * @param port path to the device file, e.g. "/dev/ttyS0", "/dev/ttyUSB0" etc
     * @return RetCode 
     */
    RetCode open(const std::string &port);

    void close();

    /**
     * @brief Write data to the serial port output buffer
     * 
     * This method is non-blocking and return code immediately. Serial port
     * driver is responsible for buffering and data transfer.
     * 
     * @param data data to be written
     * @param length length of data to be written
     * @return RetCode returns the following codes
     *     SUCCESS - data written successfully, 
     *     PORT_NOT_OPENED - method open(...) not called, 
     *     NOT_ALL_WRITTEN - not all data was sended, may be returned when the 
     * driver buffer is full, 
     *     UNDEFINED_ERROR - undefined error.
     */
    RetCode write(const void *data, size_t length);

    /**
     * @brief Write data to the serial port output buffer
     * 
     * This method is non-blocking and return code immediately. Serial port
     * driver is responsible for buffering and data transfer.
     * 
     * @param data data to be written
     * @param length length of data to be written
     * @return RetCode returns the following codes
     *     SUCCESS - data written successfully, 
     *     PORT_NOT_OPENED - method open(...) not called, 
     *     NOT_ALL_WRITTEN - not all data was sended, may be returned when the 
     * driver buffer is full,
     *     UNDEFINED_ERROR - undefined error.
     */
    RetCode write(const char *data, size_t length);

    /***************************************************************************
     * Events
     **************************************************************************/

    Event<void *, size_t> onData;
    Event<RetCode> onError;

    /***************************************************************************
     * Accessors / Mutators
     **************************************************************************/

    BaudRate baudRate() const;
    void setBaudRate(BaudRate baudRate);

    DataBits dataBits() const;
    void setDataBits(DataBits dataBits);

    Parity parity() const;
    void setParity(Parity parity);

    StopBits stopBits() const;
    void setStopBits(StopBits stopBits);

protected:
    int m_fd;
    BaudRate m_baudRate;
    DataBits m_dataBits;
    Parity m_parity;
    StopBits m_stopBits;
    termios m_options;

    static std::thread *m_pollingThread;    // singleton thread to polling ports
    static bool m_threadRunning;            // thread quit confition
    static std::mutex m_mutex;               // vectors protection mutex
    static std::vector<pollfd> m_pollFds;   // polling file descriptors
    static std::vector<SerialPort *> m_instances;    // port instances (non-static objects)

    static void polling();
};

enum class SerialPort::BaudRate {
    BR_0 = 0,
    BR_50 = 50,
    BR_75 = 75,
    BR_110 = 110,
    BR_134 = 134,
    BR_150 = 150,
    BR_200 = 200,
    BR_300 = 300,
    BR_600 = 600,
    BR_1200 = 1200,
    BR_1800 = 1800,
    BR_2400 = 2400,
    BR_4800 = 4800,
    BR_9600 = 9600,
    BR_19200 = 19200,
    BR_38400 = 38400,
    BR_57600 = 57600,
    BR_115200 = 115200
};

enum class SerialPort::DataBits {
    DB_5 = 5,
    DB_6 = 6,
    DB_7 = 7,
    DB_8 = 8
};

enum class SerialPort::Parity {
    NONE,
    EVEN,
    ODD,
    SPACE
};

enum class SerialPort::StopBits {
    ONE,
    TWO
};

/** Return codes enumeration */
enum class SerialPort::RetCode {
    SUCCESS = 0,                       /**< Successful operation */
    ALREADY_OPENED = 1,
    DEVICE_NOT_CONNECTED = 2,
    NOT_ALL_WRITTEN = 3,
    PORT_NOT_OPENED = 4,
    DEVICE_REMOVED_DURING_OPERATION = 5,
    READ_ERROR = 6,
    BUFFER_FLUSH_ERROR = 7,
    SWITCH_TO_NON_BLOCKING_MODE_ERROR = 8,
    FAILED_TO_GET_PORT_OPTIONS = 9,
    FAILED_TO_SET_PORT_OPTIONS = 10,
    UNDEFINED_ERROR
};

/*******************************************************************************
 * Inline
 ******************************************************************************/

inline SerialPort::RetCode SerialPort::write(const char *data, size_t length) { 
    return write(static_cast<const void *>(data), length); 
}

inline SerialPort::BaudRate SerialPort::baudRate() const { return m_baudRate; }
inline void SerialPort::setBaudRate(SerialPort::BaudRate baudRate) { m_baudRate = baudRate; }

inline SerialPort::DataBits SerialPort::dataBits() const { return m_dataBits; }
inline void SerialPort::setDataBits(SerialPort::DataBits dataBits) { m_dataBits = dataBits; }

inline SerialPort::Parity SerialPort::parity() const { return m_parity; }
inline void SerialPort::setParity(SerialPort::Parity parity) { m_parity = parity; }

inline SerialPort::StopBits SerialPort::stopBits() const { return m_stopBits; }
inline void SerialPort::setStopBits(SerialPort::StopBits stopBits) { m_stopBits = stopBits; }

} // namespace Hlk

#endif // HLK_SERIAL_PORT