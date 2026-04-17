/**
 * @file screen_manager.cpp
 * Two-page brain UI with category-filtered auton selector.
 *
 * Page 1 — SELECT:  category tabs (ALL/MATCH/SKILLS/TEST) + scrollable list
 *                    with color-coded category bars + prev/next navigation
 * Page 2 — INFO:    minimap + odom readout + motor temps + battery
 */
#include "ui/screen_manager.hpp"
#include "ui/field_display.hpp"
#include "ui/theme.hpp"
#include "EZ-Template/sdcard.hpp"

#include "pros/screen.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ScreenManager {

namespace {

// ── Pages ────────────────────────────────────────────────────────────────────
enum class Page { SELECT, INFO };
static Page s_page     = Page::SELECT;
static Page s_pagePrev = Page::SELECT;
static bool s_dirty    = true;
static uint32_t s_lastTouch = 0;
static ViewModel s_lastRenderedVm{};
static bool s_hasLastRenderedVm = false;

// ── Category filter state ────────────────────────────────────────────────────
enum class CatFilter { ALL, MATCH, SKILLS, TEST };
static CatFilter s_filter = CatFilter::ALL;
static std::vector<AutonEntry> s_entries;
static std::vector<int> s_filteredIndices;  // maps filtered row → absolute auton index

void rebuildFilter() {
    s_filteredIndices.clear();
    if (s_entries.empty()) {
        // No category info — show all autons from EZ-Template selector
        int count = ez::as::auton_selector.auton_count;
        for (int i = 0; i < count; i++)
            s_filteredIndices.push_back(i);
        return;
    }
    for (int i = 0; i < static_cast<int>(s_entries.size()); i++) {
        bool pass = false;
        switch (s_filter) {
            case CatFilter::ALL:    pass = true; break;
            case CatFilter::MATCH:  pass = (s_entries[i].category == AutonCategory::MATCH); break;
            case CatFilter::SKILLS: pass = (s_entries[i].category == AutonCategory::SKILLS); break;
            case CatFilter::TEST:   pass = (s_entries[i].category == AutonCategory::TEST); break;
        }
        if (pass) s_filteredIndices.push_back(i);
    }
}

uint32_t categoryColor(AutonCategory cat) {
    switch (cat) {
        case AutonCategory::MATCH:  return UITheme::kAccentMatch;
        case AutonCategory::SKILLS: return UITheme::kAccentSkills;
        case AutonCategory::TEST:   return UITheme::kAccentTest;
    }
    return UITheme::kDimGray;
}

const char* categoryLabel(AutonCategory cat) {
    switch (cat) {
        case AutonCategory::MATCH:  return "MATCH";
        case AutonCategory::SKILLS: return "SKILLS";
        case AutonCategory::TEST:   return "TEST";
    }
    return "";
}

// ── Touch regions ────────────────────────────────────────────────────────────

// Header tabs
static constexpr UITheme::Rect TAB_SELECT = {140, 2, 260, 20};
static constexpr UITheme::Rect TAB_INFO   = {268, 2, 388, 20};

// Category filter tabs (below header)
static constexpr int CAT_Y0 = 25;
static constexpr int CAT_Y1 = 47;
static constexpr UITheme::Rect CAT_ALL    = {12,  CAT_Y0, 66,  CAT_Y1};
static constexpr UITheme::Rect CAT_MATCH  = {72,  CAT_Y0, 142, CAT_Y1};
static constexpr UITheme::Rect CAT_SKILLS = {148, CAT_Y0, 224, CAT_Y1};
static constexpr UITheme::Rect CAT_TEST   = {230, CAT_Y0, 290, CAT_Y1};

// SELECT page buttons (right panel)
static constexpr UITheme::Rect BTN_PREV = {336, 56, 460, 86};
static constexpr UITheme::Rect BTN_NEXT = {336, 94, 460, 124};

// Auton list geometry
static constexpr int LIST_X0      = 12;
static constexpr int LIST_X1      = 320;
static constexpr int LIST_Y0      = 52;
static constexpr int LIST_ROW_H   = 28;
static constexpr int LIST_VISIBLE = 6;

// INFO page buttons
static constexpr UITheme::Rect BTN_ZERO_REL = {16, 178, 146, 202};

inline bool inside(const UITheme::Rect& r, int x, int y) {
    return x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1;
}

// ── Scroll & relative frame state ────────────────────────────────────────────
static int s_scrollOffset = 0;
static bool s_relativeFrameActive = false;
static float s_relativeOriginX     = 0.0f;
static float s_relativeOriginY     = 0.0f;
static float s_relativeOriginTheta = 0.0f;
static float s_lastAbsX     = 0.0f;
static float s_lastAbsY     = 0.0f;
static float s_lastAbsTheta = 0.0f;

struct DisplayPose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

double wrapDeg(double angle) {
    while (angle > 180.0)   angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

int quantizeTenths(float value) {
    return static_cast<int>(std::lround(value * 10.0f));
}

int quantizeWhole(float value) {
    return static_cast<int>(std::lround(value));
}

bool sameViewModel(const ViewModel& lhs, const ViewModel& rhs) {
    return quantizeTenths(lhs.odomX) == quantizeTenths(rhs.odomX) &&
           quantizeTenths(lhs.odomY) == quantizeTenths(rhs.odomY) &&
           quantizeTenths(lhs.odomTheta) == quantizeTenths(rhs.odomTheta) &&
           std::strncmp(lhs.autonName, rhs.autonName, sizeof(lhs.autonName)) == 0 &&
           lhs.autonIndex == rhs.autonIndex &&
           lhs.autonCount == rhs.autonCount &&
           quantizeWhole(lhs.batteryPct) == quantizeWhole(rhs.batteryPct) &&
           quantizeTenths(lhs.batteryVolts) == quantizeTenths(rhs.batteryVolts) &&
           quantizeWhole(lhs.motorTempMax) == quantizeWhole(rhs.motorTempMax) &&
           std::strncmp(lhs.hotMotor, rhs.hotMotor, sizeof(lhs.hotMotor)) == 0 &&
           std::strncmp(lhs.status, rhs.status, sizeof(lhs.status)) == 0 &&
           lhs.compConnected == rhs.compConnected &&
           lhs.imuCalibrated == rhs.imuCalibrated;
}

DisplayPose toDisplayPose(float absX, float absY, float absTheta) {
    if (!s_relativeFrameActive) return {absX, absY, absTheta};
    const double dx  = static_cast<double>(absX) - s_relativeOriginX;
    const double dy  = static_cast<double>(absY) - s_relativeOriginY;
    const double rad = static_cast<double>(s_relativeOriginTheta) * 3.14159265358979323846 / 180.0;
    const double cosA = std::cos(rad);
    const double sinA = std::sin(rad);
    return {
        static_cast<float>((dx * cosA) - (dy * sinA)),
        static_cast<float>((dx * sinA) + (dy * cosA)),
        static_cast<float>(wrapDeg(static_cast<double>(absTheta) - s_relativeOriginTheta)),
    };
}

// ── Selection helpers ────────────────────────────────────────────────────────

void selectAutonPage(int index) {
    int count = ez::as::auton_selector.auton_count;
    if (count <= 0) return;
    if (index < 0)     index = count - 1;
    if (index >= count) index = 0;
    ez::as::auton_selector.last_auton_page_current = ez::as::auton_selector.auton_page_current;
    ez::as::auton_selector.auton_page_current = index;
    ez::as::auto_sd_update();
}

int findInFiltered(int absoluteIdx) {
    for (int i = 0; i < static_cast<int>(s_filteredIndices.size()); i++) {
        if (s_filteredIndices[i] == absoluteIdx) return i;
    }
    return -1;
}

void ensureVisible(int absoluteIdx) {
    int fi = findInFiltered(absoluteIdx);
    if (fi < 0) return;
    int filteredCount = static_cast<int>(s_filteredIndices.size());
    if (fi < s_scrollOffset)                    s_scrollOffset = fi;
    if (fi >= s_scrollOffset + LIST_VISIBLE)    s_scrollOffset = fi - LIST_VISIBLE + 1;
    s_scrollOffset = std::max(0, s_scrollOffset);
    int maxOff = std::max(0, filteredCount - LIST_VISIBLE);
    if (s_scrollOffset > maxOff) s_scrollOffset = maxOff;
}

void navigateFiltered(int direction) {
    if (s_filteredIndices.empty()) return;
    int current = ez::as::auton_selector.auton_page_current;
    int fi = findInFiltered(current);
    int newFi;
    if (fi < 0) {
        // Current selection not in filter — jump to first/last
        newFi = (direction > 0) ? 0 : static_cast<int>(s_filteredIndices.size()) - 1;
    } else {
        newFi = fi + direction;
        int sz = static_cast<int>(s_filteredIndices.size());
        if (newFi < 0)   newFi = sz - 1;
        if (newFi >= sz)  newFi = 0;
    }
    selectAutonPage(s_filteredIndices[newFi]);
}

// ── Touch handler ────────────────────────────────────────────────────────────

void handleTouch() {
    auto t = pros::screen::touch_status();
    if (t.touch_status != pros::E_TOUCH_PRESSED) return;

    uint32_t now = pros::millis();
    if (now - s_lastTouch < 180) return;
    s_lastTouch = now;

    int x = t.x, y = t.y;

    // Header tabs
    if (y <= UITheme::kHeaderH) {
        if (inside(TAB_SELECT, x, y)) { s_page = Page::SELECT; return; }
        if (inside(TAB_INFO, x, y))   { s_page = Page::INFO;   return; }
        return;
    }

    // ── SELECT page touch ────────────────────────────────────────────────────
    if (s_page == Page::SELECT) {
        // Category filter tabs
        if (y >= CAT_Y0 && y <= CAT_Y1) {
            CatFilter prev = s_filter;
            if (inside(CAT_ALL, x, y))    s_filter = CatFilter::ALL;
            if (inside(CAT_MATCH, x, y))  s_filter = CatFilter::MATCH;
            if (inside(CAT_SKILLS, x, y)) s_filter = CatFilter::SKILLS;
            if (inside(CAT_TEST, x, y))   s_filter = CatFilter::TEST;
            if (s_filter != prev) {
                rebuildFilter();
                s_scrollOffset = 0;
                ensureVisible(ez::as::auton_selector.auton_page_current);
                s_dirty = true;
            }
            return;
        }

        // PREV / NEXT
        if (inside(BTN_PREV, x, y)) { navigateFiltered(-1); return; }
        if (inside(BTN_NEXT, x, y)) { navigateFiltered(1);  return; }

        // List row tap
        for (int row = 0; row < LIST_VISIBLE; row++) {
            int ry = LIST_Y0 + row * LIST_ROW_H;
            if (x >= LIST_X0 && x <= LIST_X1 && y >= ry && y < ry + LIST_ROW_H) {
                int filteredIdx = s_scrollOffset + row;
                if (filteredIdx >= 0 &&
                    filteredIdx < static_cast<int>(s_filteredIndices.size())) {
                    selectAutonPage(s_filteredIndices[filteredIdx]);
                }
                return;
            }
        }
    }

    // ── INFO page touch ──────────────────────────────────────────────────────
    if (s_page == Page::INFO) {
        if (inside(BTN_ZERO_REL, x, y)) {
            s_relativeFrameActive = true;
            s_relativeOriginX     = s_lastAbsX;
            s_relativeOriginY     = s_lastAbsY;
            s_relativeOriginTheta = s_lastAbsTheta;
            FieldDisplay::clearTrail();
            s_dirty = true;
            return;
        }
    }
}

// ── Draw helpers ─────────────────────────────────────────────────────────────

void drawTabs(Page active) {
    UITheme::text(pros::E_TEXT_SMALL, 8, 6, UITheme::kWhite, UITheme::kBgHeader, "BIGGA 6");
    UITheme::drawBtn(TAB_SELECT, "SELECT", active == Page::SELECT);
    UITheme::drawBtn(TAB_INFO,   "INFO",   active == Page::INFO);
}

void drawCatTab(const UITheme::Rect& r, const char* label,
                bool active, uint32_t accentColor) {
    uint32_t bg      = active ? UITheme::kTabActiveBg : UITheme::kBg;
    uint32_t textCol = active ? UITheme::kWhite       : UITheme::kDimGray;
    UITheme::fill(r, bg);
    UITheme::textCenter(pros::E_TEXT_SMALL, r,
                        r.y0 + (UITheme::height(r) - 12) / 2,
                        textCol, bg, "%s", label);
    if (active) {
        // 2 px accent underline
        UITheme::hline(r.x0, r.x1, r.y1,     accentColor);
        UITheme::hline(r.x0, r.x1, r.y1 - 1, accentColor);
    }
}

void drawCategoryBar() {
    // Clear category bar area
    UITheme::fill(UITheme::makeRect(0, UITheme::kHeaderH + 1,
                                    UITheme::kScreenW, CAT_Y1 - UITheme::kHeaderH),
                  UITheme::kBg);

    drawCatTab(CAT_ALL,    "ALL",    s_filter == CatFilter::ALL,    UITheme::kWhite);
    drawCatTab(CAT_MATCH,  "MATCH",  s_filter == CatFilter::MATCH,  UITheme::kAccentMatch);
    drawCatTab(CAT_SKILLS, "SKILLS", s_filter == CatFilter::SKILLS, UITheme::kAccentSkills);
    drawCatTab(CAT_TEST,   "TEST",   s_filter == CatFilter::TEST,   UITheme::kAccentTest);

    UITheme::hline(0, UITheme::kScreenW - 1, CAT_Y1 + 2, UITheme::kDivider);
}

// ── SELECT page ──────────────────────────────────────────────────────────────

void drawSelectPage(const ViewModel& vm) {
    // Clear body area
    UITheme::fill(UITheme::makeRect(0, UITheme::kBodyY,
                                    UITheme::kScreenW, UITheme::kScreenH - UITheme::kBodyY),
                  UITheme::kBg);

    drawCategoryBar();

    int filteredCount = static_cast<int>(s_filteredIndices.size());
    int current       = ez::as::auton_selector.auton_page_current;

    ensureVisible(current);

    // ── Auton list ───────────────────────────────────────────────────────────
    if (filteredCount == 0) {
        UITheme::text(pros::E_TEXT_MEDIUM, LIST_X0 + 16, LIST_Y0 + 40,
                      UITheme::kDimGray, UITheme::kBg, "No autons in category");
    } else {
        for (int row = 0; row < LIST_VISIBLE; row++) {
            int fi = s_scrollOffset + row;
            int ry = LIST_Y0 + row * LIST_ROW_H;

            if (fi >= filteredCount) {
                // Empty row past end of list
                UITheme::fill(UITheme::makeRect(LIST_X0, ry,
                              LIST_X1 - LIST_X0, LIST_ROW_H - 1), UITheme::kBg);
                continue;
            }

            int absIdx   = s_filteredIndices[fi];
            bool selected = (absIdx == current);
            uint32_t rowBg   = selected ? UITheme::kHighlight : UITheme::kBg;
            uint32_t rowText = selected ? UITheme::kWhite     : UITheme::kGray;

            // Row background
            UITheme::fill(UITheme::makeRect(LIST_X0, ry,
                          LIST_X1 - LIST_X0, LIST_ROW_H - 1), rowBg);

            // Category color bar (left edge, 4 px wide)
            if (absIdx < static_cast<int>(s_entries.size())) {
                uint32_t catCol = categoryColor(s_entries[absIdx].category);
                UITheme::fill({LIST_X0, ry, LIST_X0 + 3, ry + LIST_ROW_H - 2}, catCol);
            }

            // Selection arrow
            if (selected) {
                UITheme::text(pros::E_TEXT_MEDIUM, LIST_X0 + 10, ry + 5,
                              UITheme::kWhite, rowBg, ">");
            }

            // Auton name
            const char* name =
                (absIdx < static_cast<int>(ez::as::auton_selector.Autons.size()))
                    ? ez::as::auton_selector.Autons[absIdx].Name.c_str()
                    : "???";
            UITheme::text(pros::E_TEXT_MEDIUM, LIST_X0 + 26, ry + 5,
                          rowText, rowBg, "%.26s", name);

            // Row divider
            UITheme::hline(LIST_X0 + 6, LIST_X1, ry + LIST_ROW_H - 1, UITheme::kDivider);
        }

        // Scrollbar (only when list is scrollable)
        if (filteredCount > LIST_VISIBLE) {
            int barH   = LIST_VISIBLE * LIST_ROW_H;
            int thumbH = std::max(8, barH * LIST_VISIBLE / filteredCount);
            int maxOff = std::max(1, filteredCount - LIST_VISIBLE);
            int thumbY = LIST_Y0 + (barH - thumbH) * s_scrollOffset / maxOff;
            UITheme::fill(UITheme::makeRect(LIST_X1 + 2, LIST_Y0, 3, barH),
                          UITheme::kBgAlt);
            UITheme::fill(UITheme::makeRect(LIST_X1 + 2, thumbY, 3, thumbH),
                          UITheme::kDarkGray);
        }
    }

    // ── Right panel ──────────────────────────────────────────────────────────
    UITheme::drawBtn(BTN_PREV, "PREV");
    UITheme::drawBtn(BTN_NEXT, "NEXT");

    UITheme::hline(336, 460, 132, UITheme::kDivider);

    // Selected auton display
    UITheme::fill(UITheme::makeRect(336, 138, 130, 80), UITheme::kBg);
    UITheme::text(pros::E_TEXT_SMALL, 336, 138, UITheme::kDimGray, UITheme::kBg, "SELECTED");
    UITheme::text(pros::E_TEXT_MEDIUM, 336, 156, UITheme::kWhite, UITheme::kBg,
                  "%.14s", vm.autonName);

    // Category badge for selected auton
    if (current >= 0 && current < static_cast<int>(s_entries.size())) {
        uint32_t catCol       = categoryColor(s_entries[current].category);
        const char* catLabel  = categoryLabel(s_entries[current].category);
        UITheme::fill({336, 178, 339, 188}, catCol);
        UITheme::text(pros::E_TEXT_SMALL, 344, 179, catCol, UITheme::kBg, "%s", catLabel);
    }

    // Index display
    int totalCount = ez::as::auton_selector.auton_count;
    UITheme::text(pros::E_TEXT_SMALL, 336, 198, UITheme::kDimGray, UITheme::kBg,
                  "%d / %d", current + 1, totalCount);

    // ── Status bar ───────────────────────────────────────────────────────────
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kScreenH - 16, UITheme::kBorderLite);
    UITheme::fill(UITheme::makeRect(0, UITheme::kScreenH - 15, UITheme::kScreenW, 15),
                  UITheme::kBgAlt);
    UITheme::text(pros::E_TEXT_SMALL, 8, UITheme::kScreenH - 12,
                  UITheme::kDimGray, UITheme::kBgAlt, "%.46s", vm.status);

    // Battery in bottom-right
    char batBuf[24];
    std::snprintf(batBuf, sizeof(batBuf), "%.1fV  %.0f%%", vm.batteryVolts, vm.batteryPct);
    uint32_t batCol = vm.batteryPct > 30.f ? UITheme::kDimGray
                    : vm.batteryPct > 15.f ? UITheme::kWarn
                                           : UITheme::kDanger;
    int batW = UITheme::textW(pros::E_TEXT_SMALL, batBuf);
    UITheme::text(pros::E_TEXT_SMALL, UITheme::kScreenW - batW - 8, UITheme::kScreenH - 12,
                  batCol, UITheme::kBgAlt, "%s", batBuf);
}

// ── INFO page ────────────────────────────────────────────────────────────────

void drawInfoPage(const ViewModel& vm) {
    const DisplayPose pose = toDisplayPose(vm.odomX, vm.odomY, vm.odomTheta);

    UITheme::fill(UITheme::makeRect(0, UITheme::kBodyY,
                                    UITheme::kScreenW, UITheme::kScreenH - UITheme::kBodyY),
                  UITheme::kBg);

    // ── Left: Minimap ────────────────────────────────────────────────────────
    constexpr int MAP_SIZE = 130;
    constexpr int MAP_X    = 16;
    constexpr int MAP_Y    = UITheme::kBodyY + 8;

    UITheme::text(pros::E_TEXT_SMALL, MAP_X, MAP_Y - 2, UITheme::kDimGray, UITheme::kBg,
                  s_relativeFrameActive ? "FIELD (REL)" : "FIELD (ABS)");
    FieldDisplay::draw(MAP_X, MAP_Y + 12, MAP_SIZE, pose.x, pose.y, pose.theta);
    UITheme::drawBtn(BTN_ZERO_REL, "ZERO HERE");
    UITheme::text(pros::E_TEXT_SMALL, MAP_X, 208, UITheme::kDimGray, UITheme::kBg,
                  s_relativeFrameActive ? "Frame: relative 0,0,0" : "Frame: absolute");

    // ── Right column: Telemetry ──────────────────────────────────────────────
    constexpr int COL_X = 164;
    constexpr int COL_W = 306;
    int yy = UITheme::kBodyY + 6;

    // Odom section
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "ODOMETRY");
    yy += 16;

    char xBuf[20], yBuf[20], hBuf[20];
    std::snprintf(xBuf, sizeof(xBuf), "%+.1f", pose.x);
    std::snprintf(yBuf, sizeof(yBuf), "%+.1f", pose.y);
    std::snprintf(hBuf, sizeof(hBuf), "%.1f",  pose.theta);

    UITheme::text(pros::E_TEXT_SMALL,  COL_X,       yy,     UITheme::kDimGray, UITheme::kBg, "X");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 20,  yy - 2, UITheme::kWhite,   UITheme::kBg, "%s in", xBuf);
    UITheme::text(pros::E_TEXT_SMALL,  COL_X + 120, yy,     UITheme::kDimGray, UITheme::kBg, "Y");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 140, yy - 2, UITheme::kWhite,   UITheme::kBg, "%s in", yBuf);

    yy += 22;
    UITheme::text(pros::E_TEXT_SMALL,  COL_X,      yy,     UITheme::kDimGray, UITheme::kBg, "HDG");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2, UITheme::kWhite,   UITheme::kBg, "%s deg", hBuf);

    // Divider
    yy += 24;
    UITheme::hline(COL_X, COL_X + COL_W - 1, yy, UITheme::kDivider);
    yy += 8;

    // Diagnostics section
    UITheme::text(pros::E_TEXT_SMALL, COL_X, yy, UITheme::kDimGray, UITheme::kBg, "DIAGNOSTICS");
    yy += 16;

    // Battery
    char batBuf[32];
    std::snprintf(batBuf, sizeof(batBuf), "%.1fV  (%.0f%%)", vm.batteryVolts, vm.batteryPct);
    uint32_t batCol = vm.batteryPct > 30.f ? UITheme::kGray
                    : vm.batteryPct > 15.f ? UITheme::kWarn
                                           : UITheme::kDanger;
    UITheme::text(pros::E_TEXT_SMALL,  COL_X,      yy,     UITheme::kDimGray, UITheme::kBg, "BAT");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2, batCol,            UITheme::kBg, "%s", batBuf);
    yy += 22;

    // Motor temps
    char tempBuf[48];
    uint32_t tempCol = vm.motorTempMax < 45.f ? UITheme::kGray
                     : vm.motorTempMax < 55.f ? UITheme::kWarn
                                              : UITheme::kDanger;
    std::snprintf(tempBuf, sizeof(tempBuf), "%.0f C", vm.motorTempMax);
    UITheme::text(pros::E_TEXT_SMALL,  COL_X,      yy,     UITheme::kDimGray,  UITheme::kBg, "TEMP");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 40, yy - 2, tempCol,            UITheme::kBg, "%s", tempBuf);
    if (vm.hotMotor[0] != '\0') {
        UITheme::text(pros::E_TEXT_SMALL, COL_X + 120, yy, UITheme::kDarkGray, UITheme::kBg,
                      "(%s)", vm.hotMotor);
    }
    yy += 22;

    // IMU status
    UITheme::text(pros::E_TEXT_SMALL,  COL_X,      yy,     UITheme::kDimGray, UITheme::kBg, "IMU");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 32, yy - 2,
                  vm.imuCalibrated ? UITheme::kGray : UITheme::kWarn, UITheme::kBg,
                  vm.imuCalibrated ? "OK" : "Calibrating...");
    yy += 22;

    // Competition status
    UITheme::text(pros::E_TEXT_SMALL,  COL_X,      yy,     UITheme::kDimGray, UITheme::kBg, "COMP");
    UITheme::text(pros::E_TEXT_MEDIUM, COL_X + 40, yy - 2,
                  vm.compConnected ? UITheme::kWhite : UITheme::kDarkGray, UITheme::kBg,
                  vm.compConnected ? "Connected" : "Not connected");

    // ── Bottom status bar — show selected auton ──────────────────────────────
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kScreenH - 16, UITheme::kBorderLite);
    UITheme::fill(UITheme::makeRect(0, UITheme::kScreenH - 15, UITheme::kScreenW, 15),
                  UITheme::kBgAlt);

    // Category color indicator + auton name
    int current = ez::as::auton_selector.auton_page_current;
    if (current >= 0 && current < static_cast<int>(s_entries.size())) {
        uint32_t catCol = categoryColor(s_entries[current].category);
        UITheme::fill({6, UITheme::kScreenH - 13, 9, UITheme::kScreenH - 4}, catCol);
    }
    UITheme::text(pros::E_TEXT_SMALL, 14, UITheme::kScreenH - 12,
                  UITheme::kDimGray, UITheme::kBgAlt, "Auton: %s", vm.autonName);
}

}  // anonymous namespace

// ── Public API ───────────────────────────────────────────────────────────────

void setAutonEntries(const std::vector<AutonEntry>& entries) {
    s_entries = entries;
    rebuildFilter();
}

void init() {
    UITheme::clearScreen();
    FieldDisplay::init();
    s_page     = Page::SELECT;
    s_pagePrev = Page::SELECT;
    s_dirty    = true;
    s_scrollOffset = 0;
    s_filter = CatFilter::ALL;
    rebuildFilter();
    s_relativeFrameActive  = false;
    s_relativeOriginX      = 0.0f;
    s_relativeOriginY      = 0.0f;
    s_relativeOriginTheta  = 0.0f;
    s_hasLastRenderedVm    = false;
}

void render(const ViewModel& vm) {
    s_lastAbsX     = vm.odomX;
    s_lastAbsY     = vm.odomY;
    s_lastAbsTheta = vm.odomTheta;
    handleTouch();

    if (s_page != s_pagePrev) {
        s_pagePrev = s_page;
        s_dirty = true;
    }

    if (!s_dirty && s_hasLastRenderedVm && sameViewModel(vm, s_lastRenderedVm)) {
        return;
    }

    if (s_dirty) {
        UITheme::clearScreen();
        s_dirty = false;
    }

    // Header bar
    UITheme::fill(UITheme::makeRect(0, 0, UITheme::kScreenW, UITheme::kHeaderH),
                  UITheme::kBgHeader);
    UITheme::hline(0, UITheme::kScreenW - 1, UITheme::kHeaderH, UITheme::kBorder);
    drawTabs(s_page);

    // Page content
    switch (s_page) {
        case Page::SELECT: drawSelectPage(vm); break;
        case Page::INFO:   drawInfoPage(vm);   break;
    }

    s_lastRenderedVm = vm;
    s_hasLastRenderedVm = true;
}

void renderBoot(float progress, const char* label) {
    if (progress < 0.f) progress = 0.f;
    if (progress > 1.f) progress = 1.f;

    UITheme::clearScreen();

    // Centered boot card
    constexpr int cardW = 280;
    constexpr int cardH = 80;
    int cx = (UITheme::kScreenW - cardW) / 2;
    int cy = (UITheme::kScreenH - cardH) / 2;

    UITheme::fill(UITheme::makeRect(cx, cy, cardW, cardH), UITheme::kBgAlt);
    UITheme::outline(UITheme::makeRect(cx, cy, cardW, cardH), UITheme::kBorder);

    UITheme::textCenter(pros::E_TEXT_LARGE,
                        UITheme::makeRect(cx, cy, cardW, 30), cy + 10,
                        UITheme::kWhite, UITheme::kBgAlt, "BIGGA 6");

    UITheme::textCenter(pros::E_TEXT_SMALL,
                        UITheme::makeRect(cx, cy, cardW, 30), cy + 36,
                        UITheme::kDimGray, UITheme::kBgAlt, "%s", label ? label : "");

    UITheme::drawProgressBar(UITheme::makeRect(cx + 20, cy + 54, cardW - 40, 10), progress);
}

bool isInfoPageActive() {
    return s_page == Page::INFO;
}

}  // namespace ScreenManager
