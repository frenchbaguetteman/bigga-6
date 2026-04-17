/**
 * @file screen_manager.hpp
 * Two-page brain screen: SELECT (category-filtered auton list) and INFO (odom + diagnostics).
 */
#pragma once

#include <vector>
#include <string>

namespace ScreenManager {

// ── Auton categories for filtered selector ───────────────────────────────────
enum class AutonCategory { MATCH, SKILLS, TEST };

struct AutonEntry {
    std::string name;
    AutonCategory category;
};

// ── View model passed from screen task each frame ────────────────────────────
struct ViewModel {
    // ── Odometry ──────────────────────────────────────────
    float odomX     = 0.0f;
    float odomY     = 0.0f;
    float odomTheta = 0.0f;

    // ── Selector ──────────────────────────────────────────
    char autonName[32] = "None";
    int autonIndex        = 0;
    int autonCount        = 0;

    // ── Diagnostics ───────────────────────────────────────
    float batteryPct     = 0.0f;   // 0–100
    float batteryVolts   = 0.0f;   // e.g. 12.6
    float motorTempMax   = 0.0f;   // hottest motor °C
    char hotMotor[16]    = "";     // name of hottest motor

    // ── Status ────────────────────────────────────────────
    char status[32] = "";          // e.g. "Competition ready"
    bool compConnected = false;
    bool imuCalibrated = true;
};

void init();
void setAutonEntries(const std::vector<AutonEntry>& entries);
void render(const ViewModel& vm);
void renderBoot(float progress, const char* label);
bool isInfoPageActive();

}  // namespace ScreenManager
