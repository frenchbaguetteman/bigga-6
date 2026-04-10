/**
 * @file field_display.cpp
 * Monochrome minimap with trail rendering.
 */
#include "ui/field_display.hpp"
#include "ui/theme.hpp"
#include "pros/screen.hpp"

#include <array>
#include <cmath>
#include <algorithm>

namespace FieldDisplay {

// ── Trail buffer ─────────────────────────────────────────────────────────────
static constexpr int TRAIL_MAX = 200;
struct Pt { int x, y; };
static std::array<Pt, TRAIL_MAX> s_trail{};
static int s_head  = 0;
static int s_count = 0;

// Field is 144 x 144 inches (VEX field)
static constexpr float FIELD_IN = 144.0f;

void init()       { s_head = s_count = 0; }
void clearTrail() { s_head = s_count = 0; }

void draw(int x0, int y0, int size, float robotX, float robotY, float robotTheta) {
    float scale = static_cast<float>(size) / FIELD_IN;
    int cx = x0 + size / 2;
    int cy = y0 + size / 2;

    auto toSX = [&](float xIn) { return cx + static_cast<int>(xIn * scale); };
    auto toSY = [&](float yIn) { return cy - static_cast<int>(yIn * scale); };
    auto clampX = [&](int v) { return std::max(x0, std::min(x0 + size - 1, v)); };
    auto clampY = [&](int v) { return std::max(y0, std::min(y0 + size - 1, v)); };

    // Field background
    UITheme::fill({x0, y0, x0 + size - 1, y0 + size - 1}, 0x00080808);
    UITheme::outline({x0, y0, x0 + size - 1, y0 + size - 1}, UITheme::kBorder);

    // Grid (every 24 inches)
    int gridStep = static_cast<int>(24.0f * scale);
    if (gridStep > 1) {
        for (int gx = cx % gridStep; gx < size; gx += gridStep)
            UITheme::vline(x0 + gx, y0 + 1, y0 + size - 2, 0x00181818);
        for (int gy = cy % gridStep; gy < size; gy += gridStep)
            UITheme::hline(x0 + 1, x0 + size - 2, y0 + gy, 0x00181818);
    }

    // Center crosshair
    UITheme::hline(cx - 3, cx + 3, cy, UITheme::kDarkGray);
    UITheme::vline(cx, cy - 3, cy + 3, UITheme::kDarkGray);

    // Add current pos to trail
    int px = clampX(toSX(robotX));
    int py = clampY(toSY(robotY));
    s_trail[s_head] = {px, py};
    s_head = (s_head + 1) % TRAIL_MAX;
    if (s_count < TRAIL_MAX) ++s_count;

    // Draw trail (fading from bright to dim)
    for (int i = 0; i < s_count - 1; ++i) {
        int i0 = (s_head - 1 - i + TRAIL_MAX) % TRAIL_MAX;
        int i1 = (s_head - 2 - i + TRAIL_MAX) % TRAIL_MAX;
        // Fade: newest = brighter gray, oldest = nearly invisible
        float t = 1.0f - static_cast<float>(i) / static_cast<float>(s_count);
        int brightness = 30 + static_cast<int>(t * 70.0f);  // 30..100
        uint32_t col = (brightness << 16) | (brightness << 8) | brightness;
        UITheme::line(s_trail[i0].x, s_trail[i0].y,
                      s_trail[i1].x, s_trail[i1].y, col);
    }

    // Draw robot dot + heading line
    UITheme::fill({px - 2, py - 2, px + 2, py + 2}, UITheme::kWhite);
    float rad = robotTheta * (3.14159265f / 180.0f);
    int hx = clampX(px + static_cast<int>(8.0f * std::cos(rad)));
    int hy = clampY(py - static_cast<int>(8.0f * std::sin(rad)));
    UITheme::line(px, py, hx, hy, UITheme::kGray);
}

}  // namespace FieldDisplay
