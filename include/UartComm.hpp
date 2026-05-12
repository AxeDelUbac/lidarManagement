#pragma once

#include <cstdint>
#include <string>
#include <termios.h>

class UartComm {
public:
    explicit UartComm(const std::string& device, int baudrate = B115200);
    ~UartComm();

    UartComm(const UartComm&) = delete;
    UartComm& operator=(const UartComm&) = delete;

    bool isOpen() const { return m_fd >= 0; }

    // Retourne le nombre d'octets envoyés, -1 si erreur
    int send(const uint8_t* data, size_t length);
    int send(const std::string& str);

    // Retourne le nombre d'octets reçus, -1 si erreur, 0 si timeout
    int receive(uint8_t* buffer, size_t maxLength, int timeoutMs = 1000);

private:
    int m_fd = -1;
};
