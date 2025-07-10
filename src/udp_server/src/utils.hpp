#pragma once

#include <cstdint>
#include <vector>

struct rgb8
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

std::map<std::string, std::string> load_env(const std::string& filename);
std::vector<rgb8> get_cmap(float gamma = 3.f);