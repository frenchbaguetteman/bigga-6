# Bigga 6

VEX V5 robot control code built on PROS and EZ-Template, with additional path-tracking controllers,
trajectory planning, GPS startup configuration, and a custom brain-screen interface.

This repository brings together autonomous motion, driver control, and robot-specific subsystems.
It is a hardware project: motor ports, geometry, gains, and routines need to match the robot.

## What is added here

- **Path tracking:** LTV and RAMSETE controller implementations, with examples alongside EZ-Template PID motion.
- **Trajectory references:** waypoint sampling and velocity constraints for multi-point paths.
- **Startup pose:** configurable GPS/IMU initialization.
- **Brain UI:** field visualization, autonomous selection, and robot status views.
- **Robot integration:** intake and pneumatic control, autonomous routines, and driver bindings.

EZ-Template provides the underlying drive, PID, odometry, autonomous-selector, and competition
infrastructure. The project-specific additions above build on that foundation.

## Build and try it

Install the [PROS toolchain](https://pros.cs.purdue.edu/). The checked-in project targets VEX V5;
its project metadata records PROS kernel 4.1.1 and LVGL 8.3.8.

```sh
git clone https://github.com/frenchbaguetteman/bigga-6.git
cd bigga-6
pros make
```

Before uploading, review [robot_config.hpp](include/robot_config.hpp) and the chassis/subsystem
setup in [main.cpp](src/main.cpp). Set the motor and sensor ports, wheel dimensions, gearing,
starting pose, and gains for your hardware. Autonomous examples are in [autons.cpp](src/autons.cpp).

With a configured V5 robot connected:

```sh
pros upload
```

## Code map

| Area | Location |
| --- | --- |
| Robot geometry, ports, and configuration | [include/robot_config.hpp](include/robot_config.hpp) |
| LTV and RAMSETE controllers | [include/controllers](include/controllers) |
| Trajectory reference planner | [include/pathing/reference_planner.hpp](include/pathing/reference_planner.hpp) |
| GPS startup pose | [include/localization/gps_init.hpp](include/localization/gps_init.hpp) |
| Competition entry points and driver control | [src/main.cpp](src/main.cpp) |
| Autonomous routines and motion examples | [src/autons.cpp](src/autons.cpp) |
| Brain-screen interface | [src/ui](src/ui) |
| Path assets | [sd-card](sd-card) |

## Current scope

This version contains robot-side code; host tests and a simulator are not included.
Controller implementations and examples are not a substitute for tuning and measuring performance
on the actual robot. No tracking-accuracy benchmark is published here yet.

## Credits and license

Built with [PROS](https://pros.cs.purdue.edu/) and
[EZ-Template](https://github.com/EZ-Robotics/EZ-Template).
For upstream API documentation and tutorials, see the
[EZ-Template documentation](https://ez-robotics.github.io/EZ-Template/).

See [LICENSE.md](LICENSE.md) for the Mozilla Public License 2.0 and retain upstream notices.
