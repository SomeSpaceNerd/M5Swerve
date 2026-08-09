#include "gpio.hpp"
#include <cerrno>
#include <cstring>

namespace bellman
{
    namespace hal
    {
        GPIO* GPIO::instance = nullptr;

        GPIO::GPIO()
        {
            m_logger = hal::Logger::GetInstance();
#ifdef __linux__
            chip_count = 0;
            chip_fds = nullptr;

            line_count = 0;
            line_names = nullptr;
            line_index = nullptr;

            active = nullptr;
            active_count = 0;

            // Discover all gpiochips
            DIR* d = opendir("/dev");
            struct dirent* e;

            if (!d) { return; }

            // Count them
            while ((e = readdir(d)) != NULL)
            {
                if (strncmp(e->d_name, "gpiochip", 8) == 0) { chip_count++; }
            }

            rewinddir(d);

            chip_fds = new int[chip_count];

            int i = 0;
            while ((e = readdir(d)) != NULL)
            {
                if (strncmp(e->d_name, "gpiochip", 8) != 0) { continue; }

                std::string path = "/dev/";
                path += e->d_name;

                int fd = open(path.c_str(), O_RDONLY);
                if (fd >= 0) { chip_fds[i++] = fd; }
            }

            chip_count = i;
            closedir(d);

            // Count all of the GPIO lines
            line_count = 0;

            for (i = 0; i < chip_count; i++)
            {
                gpiochip_info info;
                memset(&info, 0, sizeof(info));

                if (ioctl(chip_fds[i], GPIO_GET_CHIPINFO_IOCTL, &info) == 0)
                {
                    line_count += info.lines;
                }
            }

            line_names = new std::string[line_count];
            line_index = new LineLoc[line_count];

            // Cache the lines for later use
            int idx = 0;

            for (i = 0; i < chip_count; i++)
            {
                gpiochip_info info;
                memset(&info, 0, sizeof(info));

                if (ioctl(chip_fds[i], GPIO_GET_CHIPINFO_IOCTL, &info) < 0) { continue; }

                unsigned int o;
                for (o = 0; o < info.lines; o++)
                {
                    gpio_v2_line_info li;
                    memset(&li, 0, sizeof(li));

                    li.offset = o;

                    if (ioctl(chip_fds[i], GPIO_V2_GET_LINEINFO_IOCTL, &li) < 0) { continue; }

                    line_names[idx] = (li.name != NULL) ? li.name : "";

                    line_index[idx].chip = i;
                    line_index[idx].offset = o;

                    idx++;
                }
            }

            line_count = idx;
#endif
        }

        GPIO::~GPIO()
        {
#ifdef __linux__
            // Close all requested GPIO line handles
            for (int i = 0; i < active_count; ++i)
            {
                if (active && active[i].handle.fd >= 0)
                {
                    close(active[i].handle.fd);
                    active[i].handle.fd = -1;
                }
            }

            // Close all gpiochip handles
            for (int i = 0; i < chip_count; ++i)
            {
                if (chip_fds && chip_fds[i] >= 0)
                {
                    close(chip_fds[i]);
                    chip_fds[i] = -1;
                }
            }

            delete[] chip_fds;
            delete[] line_names;
            delete[] line_index;
            delete[] active;

            chip_fds = nullptr;
            line_names = nullptr;
            line_index = nullptr;
            active = nullptr;

            chip_count = 0;
            line_count = 0;
            active_count = 0;

            if (instance == this)
            {
                instance = nullptr;
            }
#endif
        }

#ifdef __linux__
        int GPIO::find_line_index(const std::string& name)
        {
            int i;

            for (i = 0; i < line_count; i++)
            {
                if (line_names[i] == name)
                    return i;
            }

            return -1;
        }

        GPIO::LineHandle& GPIO::get_handle(int line_idx, LineMode mode)
        {
            int i;

            for (i = 0; i < active_count; i++)
            {
                if (active[i].chip == line_index[line_idx].chip && active[i].offset == line_index[line_idx].offset && active[i].mode == mode)
                {
                    return active[i].handle;
                }
            }

            gpio_v2_line_request req;
            memset(&req, 0, sizeof(req));

            req.num_lines = 1;
            req.offsets[0] = line_index[line_idx].offset;

            strncpy(req.consumer, "fb-hal-gpio", sizeof(req.consumer) - 1);

            req.config.flags = (mode == LineMode::kOutput) ? GPIO_V2_LINE_FLAG_OUTPUT : GPIO_V2_LINE_FLAG_INPUT;

            int chip_fd = chip_fds[line_index[line_idx].chip];

            if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0)
            {
                m_logger->LogMessage("GPIO", std::string("GPIO request failed: ") + strerror(errno), Logger::Level::kError);
                static LineHandle invalid_handle;
                invalid_handle.fd = -1;
                return invalid_handle;
            }

            ActiveLine* new_active = new ActiveLine[active_count + 1];

            for (i = 0; i < active_count; i++)
                new_active[i] = active[i];

            new_active[active_count].chip = line_index[line_idx].chip;
            new_active[active_count].offset = line_index[line_idx].offset;
            new_active[active_count].mode = mode;
            new_active[active_count].handle.fd = req.fd;

            delete[] active;
            active = new_active;
            active_count++;

            return active[active_count - 1].handle;
        }

#endif

        GPIO* GPIO::GetInstance()
        {
            if (instance == nullptr)
            {
                instance = new GPIO();
            }
            return instance;
        }

        void GPIO::SetPin(std::string name, bool state)
        {
#ifdef __linux__
            int idx = find_line_index(name);
            if (idx < 0) { return; }

            LineHandle& h = get_handle(idx, LineMode::kOutput);

            gpio_v2_line_values v;
            memset(&v, 0, sizeof(v));

            v.mask = 1;
            v.bits = state ? 1 : 0;

            if (ioctl(h.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) < 0)
            {
                m_logger->LogMessage("GPIO", std::string("SetPin ioctl failed: ") + strerror(errno), Logger::Level::kError);
            }
#endif
        }

        bool GPIO::GetPin(std::string name)
        {
#ifdef __linux__
            int idx = find_line_index(name);
            if (idx < 0)
                return false;

            LineHandle& h = get_handle(idx, LineMode::kInput);

            gpio_v2_line_values v;
            memset(&v, 0, sizeof(v));

            v.mask = 1;

            if (ioctl(h.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0)
            {
                m_logger->LogMessage("GPIO", std::string("GetPin ioctl failed: ") + strerror(errno), Logger::Level::kError);
                return false;
            }

            return (v.bits & 1) != 0;
#else
            return false;
#       endif
        }

    }
}