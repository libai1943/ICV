//
// Created by yenkn on 2021/2/5.
//

#include "common/util/color.h"

#include <cmath>

namespace common {
namespace util {

Color Color::Black(0.0, 0.0, 0.0);
Color Color::Grey(0.3, 0.3, 0.3);
Color Color::White(1.0, 1.0, 1.0);
Color Color::Red(1.0, 0.0, 0.0);
Color Color::Green(0.0, 1.0, 0.0);
Color Color::Blue(0.0, 0.0, 1.0);
Color Color::Cyan(0.0, 1.0, 1.0);
Color Color::Yellow(1.0, 1.0, 0.0);
Color Color::Magenta(1.0, 0.0, 1.0);
Color Color::Orange(1.0, 0.55, 0.0);
Color Color::Brown(0.78, 0.5, 0.3);
Color Color::Maroon(0.50, 0.0, 0.0);
Color Color::Golden(1.0, 0.84, 0.0);
std::vector<Color> Color::colors = {Color::White,  Color::Blue,  Color::Grey,   Color::Red,    Color::Green,
                                    Color::Orange, Color::Brown, Color::Maroon, Color::Magenta};

Color Color::Black_y(0.0, 0.0, 0.0, 0.5);
Color Color::Grey_y(0.3, 0.3, 0.3, 0.5);
Color Color::White_y(1.0, 1.0, 1.0, 0.5);
Color Color::Red_y(1.0, 0.0, 0.0, 0.5);
Color Color::Green_y(0.0, 1.0, 0.0, 0.5);
Color Color::Blue_y(0.0, 0.0, 1.0, 0.5);
Color Color::Cyan_y(0.0, 1.0, 1.0, 0.5);
Color Color::Yellow_y(1.0, 1.0, 0.0, 0.5);
Color Color::Magenta_y(1.0, 0.0, 1.0, 0.5);
Color Color::Orange_y(1.0, 0.55, 0.0, 0.5);
Color Color::Brown_y(0.78, 0.5, 0.3, 0.5);
Color Color::Maroon_y(0.50, 0.0, 0.0, 0.5);
Color Color::Golden_y(1.0, 0.84, 0.0, 0.5);
std::vector<Color> Color::colors_y = {Color::White_y,  Color::Blue_y,  Color::Grey_y,   Color::Red_y,    Color::Green_y,
                                      Color::Orange_y, Color::Brown_y, Color::Maroon_y, Color::Magenta_y};

Color Color::fromHSV(int H, double S, double V) {
    double C = S * V;
    double X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
    double m = V - C;
    double Rs, Gs, Bs;

    if (H >= 0 && H < 60) {
        Rs = C;
        Gs = X;
        Bs = 0;
    } else if (H >= 60 && H < 120) {
        Rs = X;
        Gs = C;
        Bs = 0;
    } else if (H >= 120 && H < 180) {
        Rs = 0;
        Gs = C;
        Bs = X;
    } else if (H >= 180 && H < 240) {
        Rs = 0;
        Gs = X;
        Bs = C;
    } else if (H >= 240 && H < 300) {
        Rs = X;
        Gs = 0;
        Bs = C;
    } else {
        Rs = C;
        Gs = 0;
        Bs = X;
    }

    return Color(Rs + m, Gs + m, Bs + m);
}

void Color::toHSV(float &fH, float &fS, float &fV) const {
    float fCMax = std::max(std::max(r_, g_), b_);
    float fCMin = std::min(std::min(r_, g_), b_);
    float fDelta = fCMax - fCMin;

    if (fDelta > 0) {
        if (fCMax == r_) {
            fH = 60 * (std::fmod(((g_ - b_) / fDelta), 6));
        } else if (fCMax == g_) {
            fH = 60 * (((b_ - r_) / fDelta) + 2);
        } else if (fCMax == b_) {
            fH = 60 * (((r_ - g_) / fDelta) + 4);
        }

        if (fCMax > 0) {
            fS = fDelta / fCMax;
        } else {
            fS = 0;
        }

        fV = fCMax;
    } else {
        fH = 0;
        fS = 0;
        fV = fCMax;
    }

    if (fH < 0) {
        fH = 360 + fH;
    }
}
}    // namespace util
}    // namespace common
