#include "utils.hpp"

#include <cmath>

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