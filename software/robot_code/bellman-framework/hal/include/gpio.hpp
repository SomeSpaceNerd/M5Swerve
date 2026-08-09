#pragma once

#include <string>
#include <logger.hpp>

#ifdef __linux__

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include <linux/gpio.h>

#endif

namespace bellman
{
    namespace hal
    {
        class GPIO
        {
        public:
            static GPIO* GetInstance();

            void SetPin(std::string name, bool state);
            bool GetPin(std::string name);

        private:
            GPIO();
            ~GPIO();

            static GPIO* instance;

            Logger* m_logger;

            #ifdef __linux__
            struct LineHandle
            {
                int fd;
            };

            enum class LineMode
            {
                kInput,
                kOutput
            };

            struct ActiveLine
            {
                int chip;
                int offset;
                LineMode mode;
                LineHandle handle;
            };

            struct LineLoc
            {
                int chip;
                int offset;
            };

            int chip_count;
            int* chip_fds;

            int line_count;
            std::string* line_names;

            LineLoc* line_index;

            ActiveLine* active;
            int active_count;

            int find_line_index(const std::string& name);
            LineHandle& get_handle(int line_index, LineMode mode);
            #endif
        };
    }
}