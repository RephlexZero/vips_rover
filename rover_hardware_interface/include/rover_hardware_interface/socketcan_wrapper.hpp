#ifndef SOCKETCAN_WRAPPER_HPP_
#define SOCKETCAN_WRAPPER_HPP_

#include <string>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdexcept>

class SocketCAN
{
public:
    SocketCAN() : sock_fd_(-1) {}

    ~SocketCAN()
    {
        if (is_open())
        {
            close();
        }
    }

    bool open(const std::string& interface_name)
    {
        sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_fd_ < 0)
        {
            return false;
        }

        ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            close();
            return false;
        }

        sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            close();
            return false;
        }
        return true;
    }

    void close()
    {
        if (sock_fd_ >= 0)
        {
            ::close(sock_fd_);
            sock_fd_ = -1;
        }
    }

    bool is_open() const
    {
        return sock_fd_ >= 0;
    }

    bool read(can_frame& frame)
    {
        if (!is_open()) return false;
        ssize_t nbytes = ::read(sock_fd_, &frame, sizeof(struct can_frame));
        return nbytes == sizeof(struct can_frame);
    }

    bool write(const can_frame& frame)
    {
        if (!is_open()) return false;
        ssize_t nbytes = ::write(sock_fd_, &frame, sizeof(struct can_frame));
        return nbytes == sizeof(struct can_frame);
    }

private:
    int sock_fd_;
};

#endif // SOCKETCAN_WRAPPER_HPP_
