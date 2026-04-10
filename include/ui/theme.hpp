/**
 * @file theme.hpp
 * Minimal monochrome UI drawing primitives for V5 brain screen.
 * Clean dark design — white/gray on dark background, no color gimmicks.
 */
#pragma once

#include "pros/screen.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace UITheme {

// ── Screen geometry ──────────────────────────────────────────────────────────
inline constexpr int kScreenW = 480;
inline constexpr int kScreenH = 240;
inline constexpr int kHeaderH = 22;
inline constexpr int kBodyY   = kHeaderH + 1;

// ── Monochrome palette ───────────────────────────────────────────────────────
inline constexpr uint32_t kBlack      = 0x00000000;
inline constexpr uint32_t kBg         = 0x00101010;
inline constexpr uint32_t kBgAlt      = 0x001A1A1A;
inline constexpr uint32_t kBgHeader   = 0x00181818;
inline constexpr uint32_t kBorder     = 0x00333333;
inline constexpr uint32_t kBorderLite = 0x00282828;
inline constexpr uint32_t kDivider    = 0x00222222;
inline constexpr uint32_t kWhite      = 0x00FFFFFF;
inline constexpr uint32_t kGray       = 0x00AAAAAA;
inline constexpr uint32_t kDimGray    = 0x00666666;
inline constexpr uint32_t kDarkGray   = 0x00444444;
inline constexpr uint32_t kHighlight  = 0x00303030;
inline constexpr uint32_t kWarn       = 0x00FF8800;
inline constexpr uint32_t kDanger     = 0x00FF3333;

// Category accent colors
inline constexpr uint32_t kAccentMatch  = 0x0044BB66;  // match green
inline constexpr uint32_t kAccentSkills = 0x00E8A820;  // skills gold
inline constexpr uint32_t kAccentTest   = 0x00668899;  // test blue-gray
inline constexpr uint32_t kTabActiveBg  = 0x00252525;  // active category tab bg

// ── Geometry helpers ─────────────────────────────────────────────────────────
struct Rect { int x0, y0, x1, y1; };

inline constexpr Rect makeRect(int x, int y, int w, int h) {
    return {x, y, x + w - 1, y + h - 1};
}
inline constexpr int  width(const Rect& r)  { return r.x1 - r.x0 + 1; }
inline constexpr int  height(const Rect& r) { return r.y1 - r.y0 + 1; }

inline Rect clamp(const Rect& r) {
    Rect o = r;
    o.x0 = std::max(0, std::min(kScreenW - 1, o.x0));
    o.y0 = std::max(0, std::min(kScreenH - 1, o.y0));
    o.x1 = std::max(0, std::min(kScreenW - 1, o.x1));
    o.y1 = std::max(0, std::min(kScreenH - 1, o.y1));
    if (o.x1 < o.x0) o.x1 = o.x0;
    if (o.y1 < o.y0) o.y1 = o.y0;
    return o;
}

// ── Text metrics ─────────────────────────────────────────────────────────────
inline int charW(pros::text_format_e_t f) {
    switch (f) {
        case pros::E_TEXT_SMALL: return 5;
        case pros::E_TEXT_LARGE:
        case pros::E_TEXT_LARGE_CENTER: return 11;
        default: return 7;
    }
}
inline int textW(pros::text_format_e_t f, const char* s) {
    return static_cast<int>(std::strlen(s)) * charW(f);
}
inline int textH(pros::text_format_e_t f) {
    switch (f) {
        case pros::E_TEXT_SMALL: return 12;
        case pros::E_TEXT_LARGE:
        case pros::E_TEXT_LARGE_CENTER: return 24;
        default: return 18;
    }
}

// ── Primitives ───────────────────────────────────────────────────────────────
inline void fill(const Rect& r, uint32_t c) {
    Rect q = clamp(r);
    pros::screen::set_pen(c);
    pros::screen::fill_rect(q.x0, q.y0, q.x1, q.y1);
}
inline void outline(const Rect& r, uint32_t c) {
    Rect q = clamp(r);
    pros::screen::set_pen(c);
    pros::screen::draw_rect(q.x0, q.y0, q.x1, q.y1);
}
inline void hline(int x0, int x1, int y, uint32_t c = kDivider) {
    pros::screen::set_pen(c);
    pros::screen::draw_line(x0, y, x1, y);
}
inline void vline(int x, int y0, int y1, uint32_t c = kDivider) {
    pros::screen::set_pen(c);
    pros::screen::draw_line(x, y0, x, y1);
}
inline void line(int x0, int y0, int x1, int y1, uint32_t c) {
    pros::screen::set_pen(c);
    pros::screen::draw_line(x0, y0, x1, y1);
}

// ── Text ─────────────────────────────────────────────────────────────────────
template<typename... A>
inline void text(pros::text_format_e_t f, int x, int y,
                 uint32_t color, uint32_t bg, const char* fmt, A... a) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), fmt, a...);
    Rect r = clamp({x - 1, y - 1, x + textW(f, buf) + 1, y + textH(f)});
    fill(r, bg);
    pros::screen::set_eraser(bg);
    pros::screen::set_pen(color);
    pros::screen::print(f, x, y, "%s", buf);
}

template<typename... A>
inline void textCenter(pros::text_format_e_t f, const Rect& r, int y,
                       uint32_t color, uint32_t bg, const char* fmt, A... a) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), fmt, a...);
    int x = r.x0 + std::max(0, (width(r) - textW(f, buf)) / 2);
    text(f, x, y, color, bg, "%s", buf);
}

// ── Widgets ──────────────────────────────────────────────────────────────────
inline void clearScreen() {
    pros::screen::set_pen(kBg);
    pros::screen::fill_rect(0, 0, kScreenW - 1, kScreenH - 1);
    pros::screen::set_eraser(kBg);
    pros::screen::erase();
}

inline void drawHeader(const char* left, const char* right = nullptr) {
    fill(makeRect(0, 0, kScreenW, kHeaderH), kBgHeader);
    hline(0, kScreenW - 1, kHeaderH, kBorder);
    text(pros::E_TEXT_SMALL, 8, 6, kWhite, kBgHeader, "%s", left);
    if (right && right[0]) {
        int rw = textW(pros::E_TEXT_SMALL, right);
        text(pros::E_TEXT_SMALL, kScreenW - rw - 8, 6, kDimGray, kBgHeader, "%s", right);
    }
}

inline void drawBtn(const Rect& r, const char* label, bool active = false) {
    uint32_t bg = active ? kHighlight : kBgAlt;
    fill(r, bg);
    outline(r, active ? kGray : kBorder);
    textCenter(pros::E_TEXT_SMALL, r, r.y0 + (height(r) - 12) / 2,
               active ? kWhite : kGray, bg, "%s", label);
}

inline void drawProgressBar(const Rect& r, float pct, uint32_t fg = kGray) {
    Rect q = clamp(r);
    if (pct < 0.f) pct = 0.f;
    if (pct > 1.f) pct = 1.f;
    fill(q, kBgAlt);
    int w = static_cast<int>((width(q) - 2) * pct);
    if (w > 0) fill({q.x0 + 1, q.y0 + 1, q.x0 + w, q.y1 - 1}, fg);
    outline(q, kBorder);
}

}  // namespace UITheme
