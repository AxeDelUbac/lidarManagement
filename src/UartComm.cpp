#include "UartComm.hpp"

#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <stdexcept>

UartComm::UartComm(const std::string& device, int baudrate)
{
    m_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0)
        throw std::runtime_error("Impossible d'ouvrir " + device);

    termios tty{};
    if (tcgetattr(m_fd, &tty) != 0)
        throw std::runtime_error("tcgetattr échoué sur " + device);

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    // 8N1, pas de contrôle de flux
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
        throw std::runtime_error("tcsetattr échoué sur " + device);
}

UartComm::~UartComm()
{
    if (m_fd >= 0)
        close(m_fd);
}

int UartComm::send(const uint8_t* data, size_t length)
{
    return static_cast<int>(write(m_fd, data, length));
}

int UartComm::send(const std::string& str)
{
    return send(reinterpret_cast<const uint8_t*>(str.c_str()), str.size());
}

int UartComm::receive(uint8_t* buffer, size_t maxLength, int timeoutMs)
{
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(m_fd, &readSet);

    timeval timeout{};
    timeout.tv_sec  = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;

    int ready = select(m_fd + 1, &readSet, nullptr, nullptr, &timeout);
    if (ready <= 0)
        return ready; // 0 = timeout, -1 = erreur

    return static_cast<int>(read(m_fd, buffer, maxLength));
}
