#pragma once

// ── EZ-Template demo / test routines ─────────────────────────────────────────
void default_constants();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

// ── LTV / RAMSETE controller examples ────────────────────────────────────────
void ramsete_move_example();
void ltv_move_example();
void ramsete_path_example();
void ltv_path_example();
void ramsete_with_pid_example();

// ── Competition routines ──────────────────────────────────────────────────────
void red_positive_auton();
void red_negative_auton();
void blue_positive_auton();
void blue_negative_auton();
void left43();
void left7wing();
void skills();
void rightAWP();
void rightActualAWP();
void shitty_skills();

// ── Dispatcher: routes selected Auton enum → correct function ─────────────────
void run_selected_auton();
