#include <string>
#include <vector>

std::vector<std::string> list_serial_devices(std::string dir = "/dev");
int open_serial(std::string path);
void apply_default_settings(int fd);
