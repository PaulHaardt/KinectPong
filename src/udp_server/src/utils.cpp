#include "utils.hpp"

#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <cstdlib>
#include <algorithm>

std::map<std::string, std::string> load_env(const std::string& filename) {
    std::ifstream file(filename);
    std::map<std::string, std::string> env_map;

    if (!file.is_open()) {
        std::cerr << "Could not open .env file\n";
        return env_map;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#') continue;

        std::istringstream is_line(line);
        std::string key;
        if (std::getline(is_line, key, '=')) {
            std::string value;
            if (std::getline(is_line, value)) {
                env_map[key] = value;

                // Optionally, set it as an environment variable
                // POSIX only (use `_putenv_s` on Windows)
                setenv(key.c_str(), value.c_str(), 1);
            }
        }
    }

    return env_map;
}

std::vector<rgb8> get_cmap(float gamma) {
    std::vector<rgb8> color_map(2048);
    for (int i = 0; i < 2048; ++i) {
        float v = i / 2048.f;
		v = powf(v, gamma) * 6;
		int pval = v*6*256;

        int lb = pval & 0xff;
        rgb8 color = {0, 0, 0};
        switch (pval>>8) {
            case 0:
                color = {(uint8_t) 255, (uint8_t) (255-lb), (uint8_t) (255-lb)};
                break;
            case 1:
                color = {(uint8_t) 255, (uint8_t) lb, (uint8_t) 0};
                break;
            case 2:
                color = {(uint8_t) (255-lb), (uint8_t) 255, (uint8_t) 0};
                break;
            case 3:
                color = {(uint8_t) 0, (uint8_t) 255, (uint8_t) lb};
                break;
            case 4:
                color = {(uint8_t) 0, (uint8_t) (255-lb), (uint8_t) 255};
                break;
            case 5:
                color = {(uint8_t) 0, (uint8_t) 0, (uint8_t) (255-lb)};
                break;
            default:
                break;
        }
        color_map[i] = color;
    }
    return color_map;
}

double getMinFromPointer(uint16_t* values, size_t length) {
    if (length == 0) return 0.0;

    std::vector<uint16_t> temp(values, values + length);
    std::sort(temp.begin(), temp.end());

    return temp[0];
}