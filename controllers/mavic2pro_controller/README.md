# Mavic 2 Pro zigzag controller

Autonomous C controller for the DJI Mavic 2 Pro in Webots. On simulation start the drone takes off, scans the rectangle in front of its launch heading with a lawnmower path, returns to the launch point, and lands.

## Add the controller

The source file is `controllers/mavic2pro_controller/mavic2pro_controller.c`. Webots loads a controller by that directory name. This project’s world already selects it:

```
controller "mavic2pro_controller"
```

To assign it by hand, open `worlds/mavic_patrol.wbt`, select the Mavic 2 PRO robot, and set the controller field to `mavic2pro_controller`. The previous controllers remain available as `mavic_joystick` and `mavic_patrol`.

## Build

From a shell where `WEBOTS_HOME` points at the Webots installation:

```
cd controllers/mavic2pro_controller
make
```

Opening the world in Webots also builds the controller. The program is C, not C++.

## Model notes

Checked against the R2025a `Mavic2Pro.proto` used by `worlds/mavic_patrol.wbt`.

Motors, shaft axis +Z, and the sign required for upward thrust:

| Device | thrustConstants[0] | Velocity command |
| --- | --- | --- |
| `front left propeller` | +0.00026 | as computed |
| `front right propeller` | -0.00026 | negated |
| `rear left propeller` | -0.00026 | negated |
| `rear right propeller` | +0.00026 | as computed |

Sensors already on the proto: `gps`, `compass`, `gyro`, `inertial unit`.

The stock proto has no accelerometer and no altimeter. The world adds both on `bodySlot`, at the robot origin, under the names `accelerometer` and `altimeter`. The controller stops with an error if either device is missing.

World frame is ENU. Heading printed in the console is 0° north, 90° east, clockwise, from the upright compass. The scan rectangle is centered on the launch heading, starts on the near edge through the launch point, and runs `AREA_LENGTH` meters forward.
