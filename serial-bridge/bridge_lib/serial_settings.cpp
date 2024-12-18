#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <rbl/std_format.h>
#include <filesystem>
namespace fs = std::filesystem;

#define CHECK(retval, msg) do{          \
    if(retval < 0) {                    \
        throw std::runtime_error(msg);  \
    }                                   \
} while (0);

#define CHECK_FALSE(retval, msg) do{          \
    if(retval) {                    \
        throw std::runtime_error(msg);  \
    }                                   \
} while (0);


std::vector<std::string> list_serial_devices(std::string dir = "/dev")
{
    std::vector<std::string> result{};
    for(const auto & entry : fs::directory_iterator(dir)) {
        auto p = entry.path().generic_string();
        if(p.find("ttyACM") != std::string::npos) { //} || p.find("ttyUSB")) {
            result.push_back(p);
        }
    }
    return result;
}
int open_serial(std::string path)
{
    int fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK /*same as O_NDELAY*/);
    if(fd < 0) {
        throw std::runtime_error(std_format("failed opening %s", path.c_str()));
    }
    if(! isatty(fd)) {
        throw std::runtime_error(std_format("isatty failed path:[%s]", path.c_str()));
    }
    return fd;
}

void apply_default_settings(int fd)
{

    struct termios config{};
    struct termios current_config{};
    CHECK((tcgetattr(fd, &current_config)), "getting currernt config")

    //
    // Input flags - Turn off input processing
    //
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    //
    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    config.c_oflag = 0;

    //
    // No line processing
    //
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    //
    // Turn off character processing
    //
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //

    // c_cflags
    // This is where a serial port used for data communication mis likely to require
    // changes as typically byte size, paity and baud are the only settings that change.
    //
    // This function is currently hardwired to 115200, 8N
    //
    //CBAUD   baud speed mask 4+1 bits not in posix only if _BSD_SOURCE or _SVID_SOURCE
    //CBAUDX  extra baud speed mask 1 bit not in posix only if _BSD_SOURCE or _SVID_SOURCE
    //CSTOPB  ON enables 2 stop bits
    //CSIZE   is the mask for baud rate
    //CREAD
    //PARENB  is the mask for parity generation on output and  checkiing on input and output
    //PAREODD when ON parity for input and output is odd OFF parity is even
    //HUPCL
    //CLOCAL  Ignore modem control lines - maybe we need this
    //LOBLK   not implemented on Linux
    //CIBAUD  not in posix only if _BSD_SOURCE or _SVID_SOURCE
    //CMSPAR  only if _BSD_SOURCE or _SVID_SOURCE
    //CRTSCTS enable TRS/CTS hardware defined in <bits/termios-baud.h> on Ubuntu 22.04
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8; // can be CS5, CS6, CS7, CS8
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 0;

    //
    // Communication speed (simple version, using the predefined
    // constants)
    //
    // Valid buad rates are defined in <sys/termios.h> which
    // #include <bits/termios.h> which
    // #include <bits/termios-c_cc.h>
    //
    // use CLION intellisense type a B and scroll down
    //
    CHECK_FALSE((cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200)), "setting baud rate with cfispeed and cfospeed")

    //
    // Finally, apply the configuration
    //
    CHECK((tcsetattr(fd, TCSAFLUSH, &config)), "applying the config")

    //
    // Ensure exclusive access
    //
    CHECK((ioctl(fd, TIOCEXCL, NULL)), "exclusive ioctl call")
}