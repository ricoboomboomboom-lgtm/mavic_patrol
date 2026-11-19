# mavic_manual

Manual position-hold controller for the Webots R2025a DJI Mavic 2 Pro model.
It is an independent cascade controller written for this simulator. It is not
DJI firmware and it does not reproduce a proprietary flight stack.

The pilot flies with a USB game controller, intended to be a RadioMaster TX12 MKII
in Mode 2. With the sticks released the controller levels the aircraft, bleeds
off horizontal speed, then holds the position and altitude where it settled.
The throttle stick does not spring back to center, so its neutral point is a
calibrated raw value, not "the middle of the range".

Out of the box the propellers cannot start. `diagnostic_mode` is 1, and the
four stick axes plus the arm and takeoff controls are unassigned (`-1`).

## What was checked in this session

Checked here:

- Clean build with the Webots R2025a Makefile (gcc 14.2.0, `-Wall -Wextra`). No warnings.
- Numeric self-checks for angle wrapping, PID saturation, mixer signs, and the horizontal speed limiter. These are not a flight.
- `worlds/mavic_manual.wbt` in Webots `--batch --mode=fast` for several simulated seconds.
  - All four propeller motors, the inertial unit, gyro, GPS, compass, the added accelerometer, the added altimeter, both LEDs, and the supervisor contact tracker were found.
  - The aircraft stayed in `DIAGNOSTIC`. Motor commands stayed at 0.
  - After the gear settled, Webots reported 4 contact points and the ground-contact test passed (`contact z <= 0.08`).
  - Resting attitude was about roll 0 and pitch -4 degrees. Altitude settled near 0.07 m. Accelerometer at rest was about `(0.7, 0, 9.8) m/s²`, so body +Z reads upward and includes gravity.
  - Webots printed `No free joystick found`. No RadioMaster was available to this process, so stick mapping, arming, takeoff, position hold, and USB failsafe were not flown.

Not checked here: closed-loop flight, gain tuning, TX12 axis order, motor response to a real stick, or failsafe with the USB cable pulled.

## Vehicle facts used by the mixer

Source: `Mavic2Pro.proto` in Webots R2025a. Frame is ENU. At yaw 0 the nose points +X and the left side points +Y. Z is up.

The inertial unit uses right-handed angles:

- positive roll is right wing down
- positive pitch is nose down
- positive yaw is nose left (counterclockwise from above)

That pitch sign matches the batch run: the model sits a few degrees nose-up (negative pitch) and the accelerometer shows a positive X component.

| Rotor | Name | Thrust constant | `wb_motor_set_velocity` sign |
|---|---|---:|---|
| Front left | `front left propeller` | +0.00026 | command as computed |
| Front right | `front right propeller` | -0.00026 | negated |
| Rear left | `rear left propeller` | -0.00026 | negated |
| Rear right | `rear right propeller` | +0.00026 | command as computed |

`maxVelocity` is 576 rad/s. The body mass in the proto is 0.4 kg and the center of mass is `(-0.088, 0, -0.04)` m, behind the geometric center, which is why pitch authority is not the same as roll.

Unsigned mix, before the velocity signs above:

```text
front left  = collective - roll + pitch - yaw
front right = collective + roll + pitch + yaw
rear left   = collective - roll - pitch + yaw
rear right  = collective + roll - pitch - yaw
```

Positive `roll` adds thrust on the right. Positive `pitch` adds thrust on the front, which picks up a nose-down attitude. Positive `yaw` yaws the nose left. Each unsigned command is clamped to `[0, motor_command_limit]` so a rotor is not driven backwards.

`hover_thrust = 68.5` rad/s is the collective from the stock `mavic2pro.c` sample for this proto. It was not measured again in this session. `motor_command_limit` defaults to 160, well under 576, until the signs have been watched in the simulator.

Stock devices are `inertial unit`, `gyro`, `gps`, and `compass`. The stock proto has no accelerometer and no altimeter. GPS also provides `wb_gps_get_speed_vector()` in metres per second.

## World changes

`worlds/mavic_manual.wbt` is a new world. `worlds/mavic_patrol.wbt` is unchanged.

| Change | Why |
|---|---|
| `controller "mavic_manual"` | Selects this controller. |
| `supervisor TRUE` | Ground contact comes from contact-point positions, not from "altitude reached a number". A point at world Z ≤ `ground_contact_z` (default 0.08 m, the arena floor is Z = 0) counts as the floor. A wall strike higher than that does not. |
| `Accelerometer` and `Altimeter` on `bodySlot` | The stock proto does not have them. The altimeter is a second altitude source if GPS is lost. The accelerometer is read and reported. It is not inside the control law, because its sign was only observed at rest. |
| `basicTimeStep 8` | 125 Hz, matching the existing Mavic worlds. |
| `defaultDamping` linear 0.5, angular 0.5 | Same as the stock Mavic demo, so the sample hover collective stays relevant. |

If you attach this controller to a Mavic that is not a supervisor, landing detection falls back to a weaker test: the aircraft must already have climbed, be back in the low band, be descending on command, and have almost no vertical or horizontal speed for 0.8 s. Reaching an altitude by itself never disarms the motors. Prefer the supervisor world.

The controller still flies on GPS alone if the body-slot sensors are missing. Compass loss does not change the control yaw. Control yaw is the inertial unit. The compass is only a disagreement flag.

## Control structure

Every `wb_robot_step()` does, in order:

1. Read sensors.
2. Estimate attitude, altitude, vertical speed, and horizontal velocity. GPS speed is preferred. Gyro can coast the attitude for up to 2 s if the inertial unit drops out.
3. Read the joystick. Invalid or disconnected input is replaced with zeros immediately. The last stick position is never held.
4. Step the state machine.
5. Position loop, then altitude loop, then attitude loop.
6. Mix four motor velocities.
7. Print diagnostics at a limited rate.

Lateral law, per axis, with a slew on the tilt targets:

- **Manual.** Stick deflection maps to a tilt target, ±25° roll and pitch. It does not map to a velocity. If horizontal speed exceeds 5 m/s the tilt target is blended toward the tilt that holds the speed at the limit. That is a constraint around the angle command, not a second stick-to-speed mapping.
- **Brake.** Sticks inside the deadzone. Tilt is driven from velocity error toward 0 m/s. Position is not latched yet, so the stop is not fought by a position error.
- **Hold.** After speed stays under `brake_speed_enter_hold` (default 0.12 m/s) for `brake_hold_time` (default 0.25 s), the current X/Y is latched. A position loop commands a velocity, limited to 5 m/s, and that velocity error becomes a tilt. Integrators freeze or leak while the velocity command is saturated.
- **Level.** Used when GPS or attitude cannot support a position loop. Wings are commanded level. Drift is then possible. The console says so.

Vertical law:

- Throttle above the calibrated deadzone commands a climb rate up to `max_ascent_speed`.
- Throttle below it commands a descent rate down to `max_descent_speed`.
- Inside the deadzone the current altitude is latched and held.
- Those three speeds are starting guesses in the config file. They are not measured Mavic performance numbers.
- Near the ground a descent command is scaled down, but it is not forced to zero, so the gear can still reach the floor.
- Collective is slewed (`collective_slew`) so the hover spool-up is not a step.

Yaw:

- Stick right commands a clockwise rate, up to `max_yaw_rate_deg`, slewed.
- Released stick slews the rate to zero, then latches the heading once the yaw rate is small.
- Heading error is wrapped to ±π.
- Position hold stays in the world frame, so a yaw does not move the X/Y target.

Attitude inner loop, for roll and pitch:

```text
rate_sp = angle_kp * (angle_sp - angle)
motor   = rate_kp * (measured_rate - rate_sp) + trim
```

With `roll_rate_kp = 1` and `roll_angle_kp = 50` this is the stock sample's roll P and rate damping, rearranged. Pitch uses the sample's P of 30 because the airframe is not symmetric. A small trim integrator runs only near level. Yaw uses the opposite rate-error sign so positive output yaws nose-left. Treat every gain below `hover_thrust` in the config as something to confirm in the simulator before raising `motor_command_limit`.

## State machine

| State | Enter when | Does | Leaves when |
|---|---|---|---|
| `INITIALIZATION` | Startup | Motors at 0, devices enabled | Motors exist and 0.2 s have passed |
| `DIAGNOSTIC` | `diagnostic_mode = 1` | Motors locked off, stick and sensor printout | Never, until you set `diagnostic_mode = 0` and restart |
| `READY` | Init finished and diagnostics are off | Wait for finite attitude, gyro, and altitude | Those are stable for 0.2 s |
| `DISARMED` | Ready, or a rejected arm, or a timeout | Motors spin down and stay off | A new rising edge of the arm control, and every arm check passes |
| `ARMING` | Arm accepted | Idle collective for `arming_time` | Checks fail (back to disarmed) or the timer elapses |
| `ARMED` | Arming finished | Idle collective, gentle attitude authority, no climb | Takeoff edge, disarm edge, joystick loss, or `armed_timeout` |
| `TAKEOFF` | Takeoff edge while armed, sticks neutral | Hold X/Y, climb to `ground_z + takeoff_height` | Height and vertical speed are quiet for 0.3 s, or USB/attitude failure |
| `FLIGHT` | Stick is commanding tilt, or position cannot be held | Manual or brake lateral law, throttle commands vertical speed | Sticks released and speed decayed, or a low descent, or a failure |
| `POSITION_HOLD` | Brake finished, or takeoff finished, or failsafe handed back | Hold the latched X/Y and, with neutral throttle, the latched altitude | Stick moves, low descent starts, or a failure |
| `LANDING` | Airborne, throttle down, and height under `landing_enter_height` | Attitude, yaw, and position stay active. Descent is softened near the floor | Throttle neutralized or reversed (back to hold/flight), touchdown, or USB loss |
| `FAILSAFE` | USB stale while airborne, or attitude lost while airborne | Sticks are ignored. Position and altitude are held only if those estimates still exist. Otherwise wings-level or open-loop collective. Then a descent. | Touchdown, or USB returns with fresh data, neutral sticks, and a healthy attitude for `handover_valid_s` |
| `LANDED` | Ground contact confirmed after the aircraft has actually climbed | Motors ramp to 0. A later USB reconnect does not arm | A new arm edge after the arm control has been released, with the same checks as the first arm |

Arming is refused unless all of these are true: diagnostics are off, an arm button or POV is assigned, the joystick is connected and the four axes are mapped, attitude and altitude are finite, the vehicle is within `arm_max_tilt_deg` of level, the ground altitude has been captured, and roll, pitch, yaw, and throttle are inside their deadzones. There is no path from `DISARMED` to `ARMED` without that edge.

Disarm in the air is ignored. Motors are not cut because USB or GPS was lost. They are cut after a confirmed touchdown, or on the ground after arming timeout / an explicit disarm.

Attitude loss while flying is not treated as a USB loss. The mixer drops the attitude terms and the collective goes to `degraded_thrust` (default 64 rad/s, slightly under the sample hover). That is an open-loop sink, not a promised safe landing. GPS loss while the link and the inertial unit are healthy leaves the pilot in angle mode and disables position hold. It does not start an automatic landing.

## Build

From the Webots text editor, open `controllers/mavic_manual/mavic_manual.c` and use the Build button. Webots sets `WEBOTS_HOME` and the MinGW path for you.

From PowerShell, gcc will not start unless the MinGW runtime DLLs are on `PATH` (`mingw64\bin\cpp` holds `libwinpthread-1.dll`):

```powershell
$env:WEBOTS_HOME = "C:\Program Files\Webots"
$env:PATH = "C:\Program Files\Webots\msys64\mingw64\bin;C:\Program Files\Webots\msys64\mingw64\bin\cpp;C:\Program Files\Webots\msys64\usr\bin;" + $env:PATH
cd C:\Users\TestUser\mavic_patrol\controllers\mavic_manual
make
```

`make clean` removes the build. The executable is `controllers/mavic_manual/mavic_manual.exe`.

## Run

1. Open `worlds/mavic_manual.wbt`.
2. Leave `diagnostic_mode = 1` for the first launch.
3. Press the Webots play button.
4. Watch the console. You want `devices ... motors=1 ... supervisor=1`, then `state -> DIAGNOSTIC`, and motor commands at 0.
5. The aircraft should sit on the arena. It must not spool up.

To fly later, edit `mavic_manual.cfg`, set `diagnostic_mode = 0`, rebuild is not required (the file is read at controller start), and reload the simulation.

## TX12 MKII

Put the radio in USB joystick mode (EdgeTX or OpenTX: USB mode Joystick, not USB serial). Windows should list it as a game controller. Webots uses one joystick, the first one it can open. Close other programs that have grabbed it. The console warning `No free joystick found` means Webots opened the API and no device was free. That is different from a missing API.

This controller uses only `wb_joystick_*` from Webots R2025a:

- `wb_joystick_enable`
- `wb_joystick_is_connected`
- `wb_joystick_get_model`
- `wb_joystick_get_number_of_axes`
- `wb_joystick_get_axis_value` (integer counts, normally -32768..32767)
- `wb_joystick_get_number_of_povs`
- `wb_joystick_get_pov_value`
- `wb_joystick_get_pressed_button` until it returns -1

The API does not report how many buttons exist. Diagnostics list the ids that are down right now. Axis order is not assumed. Mode 2 is the pilot convention after you map the indices:

| Stick | After mapping, positive command |
|---|---|
| Right horizontal | Roll right |
| Right vertical | Pitch forward (nose down) |
| Left horizontal | Yaw right (clockwise) |
| Left vertical | Climb |

The left vertical stick will stay where you leave it. Park it at the detent you want to call "hold altitude" before you copy `throttle_neutral`.

### Calibration

1. Start in diagnostic mode with the radio connected.
2. Move one stick fully both ways, then release it (park the throttle at the detent).
3. Read `axNN raw / min / max` and the direction (`rising` or `falling`).
4. The index that changed is that function's `*_axis`.
5. If the physical sense is backwards, set `*_invert = 1` rather than swapping the control law.
6. Copy `min`, `max`, and the raw value at the released detent into `*_min`, `*_max`, and `*_neutral`.
7. Press the switch you want for arm and write its printed id into `arm_button`. Do the same for `takeoff_button`. Use a third id for `disarm_button` if you want a ground disarm. Leave anything you have not identified at `-1`.
8. A hat can be used instead of a button: set `arm_pov` to the POV index and `arm_pov_value` to the integer printed while you hold that direction. `-1` disables it.
9. Set `diagnostic_mode = 0` and restart the simulation.
10. Do not arm until the console shows finite sensors, `ground=1`, and `safe=1` with the sticks parked. `safe=1` means all four mapped channels are inside their deadzones.

Deadzones, expo, and the low-pass times are in the same file. Expo is 0 on the throttle so the climb rate stays proportional.

If Windows sees the TX12 and Webots still says no joystick, the usual causes are USB mode still set to serial, or another program holding the device. A second input stack (DirectInput or Raw Input, both already on Windows 11, no extra library) is only worth adding after that check fails. It is not included here, because the Webots joystick API did load and report an empty device list.

## Flying

1. Sticks at neutral, including the throttle detent. Aircraft level. Console not listing an arm block.
2. Press arm once. The state becomes `ARMING`, then `ARMED`. The props spin at `idle_thrust` (default 20 rad/s), which is far under the sample hover of 68.5. Nothing should lift.
3. Press takeoff once. It is a separate edge, so holding the arm switch through arming does not take off even if you assigned the same button to both. The climb target is 1 m above the captured ground altitude, at `takeoff_climb_speed`.
4. At the top it enters `POSITION_HOLD`. Neutral throttle holds height. Neutral right stick holds position after the brake. Left stick yaws and then holds heading.
5. To land, hold the throttle below the detent. Under `landing_enter_height` the state becomes `LANDING`. Keep the stick down until the console says ground contact and `LANDED`. Centering the throttle above that gate holds altitude instead of landing.
6. After `LANDED`, release the arm control. The next arm press is required before the motors may start again. Plugging USB back in does not arm.

LEDs: diagnostic blinks slowly, armed is steady, flight alternates, landing blinks slowly, failsafe blinks quickly, disarmed and landed are off.

## Failsafe

USB is stale when `wb_joystick_is_connected()` is false, or when mapped axes are invalid for `usb_timeout_s` (default 0.2 s). Stick outputs are already zero on the first bad sample. On the ground that disarms. In the air it enters `FAILSAFE`:

- previous stick commands are not applied
- attitude stays in the loop if the inertial unit or the short gyro coast is valid
- X/Y is held if GPS is valid
- a descent at `failsafe_descent_speed` starts, softened near the floor
- motors stay on until the contact test passes

Handover back to the pilot requires a fresh joystick, every stick in its deadzone (including the throttle detent), a valid attitude, and `handover_valid_s` of that condition. The altitude and position are latched at the handover so the targets do not jump. If the sticks are not neutral, the descent continues and the console says why.

If the aircraft has already reached `LANDED` or `DISARMED`, restoring USB does nothing to the motors.

GPS loss, compass loss, gyro loss, inertial-unit loss, altimeter loss, and accelerometer loss are separate flags in the telemetry (`faults`). Only loss of attitude while airborne, or loss of USB while airborne, forces `FAILSAFE`. Loss of both altitude sources while the attitude is fine uses `degraded_thrust` and does not pretend to hold height.

## Tuning

Change `controllers/mavic_manual/mavic_manual.cfg` and restart the simulation. No rebuild.

Do this in order, with `motor_command_limit` still at 160:

1. In `ARMED`, confirm the idle spin does not lift the gear. Then take off only after that.
2. Watch a small roll and a small pitch. Right stick right must move the aircraft right. Stick forward must move it forward. If a whole axis is backwards, flip that axis's `*_invert`. If one motor is backwards, stop and fix the mixer; do not invert the stick to hide it.
3. Yaw stick right must rotate clockwise as seen from above.
4. Neutral sticks must return to level, slow down, then hold. If it oscillates, lower `roll_angle_kp` / `pitch_angle_kp` before touching anything else. If it is sluggish and does not hold a tilt, raise `roll_rate_kp` / `pitch_rate_kp` slightly. The product `angle_kp * rate_kp` is the effective angle gain (50 and 30 in the sample).
5. If hover climbs or sinks with neutral throttle, change `hover_thrust` in small steps around 68.5.
6. Then adjust `max_ascent_speed`, `max_descent_speed`, `takeoff_climb_speed`, and `failsafe_descent_speed`.
7. Position hold: `pos_kp` sets how fast a metre of error becomes a metres-per-second command. `vel_to_tilt` sets how many radians of tilt a 1 m/s velocity error requests. `tilt_slew_deg_s` limits how fast that tilt can change.
8. Raise `motor_command_limit` toward 400 only after the signs and the hover collective look right. The proto allows 576.

`roll_trim_ki` and `pitch_trim_ki` exist to hold a small angle bias. Set them to 0 if the aircraft hunts while sitting still in a hover.

## Test plan

Each item lists the expected result. Items not marked as already seen still have to be done in an interactive session with the radio connected.

### 1. Build

- `make` finishes and writes `mavic_manual.exe`. Already seen, no warnings.
- Reloading the world starts this controller, not `mavic2pro_controller`.

### 2. Sensors, no radio required

- Console lists imu, gyro, gps, compass, accel, altimeter, motors, supervisor all 1. Already seen.
- Sitting still: roll near 0, pitch a few degrees nose-up, yaw stable, `z` near 0.07–0.10, `vz` near 0, `ground=1`. Already seen after the bounce settled.
- `contact` greater than 0 and `ground_contact=1` while sitting, motors at 0. Already seen.
- Pick the aircraft up in the supervisor (or let a later flight climb) and confirm `ground_contact` returns to 0. Not done.

### 3. Radio

- Connect the TX12 in USB joystick mode. `connected=1`, model string non-empty, `axes` at least 4.
- Move one stick at a time and record the axis index, min, max, and the sign.
- Park the throttle at the detent and copy that raw value to `throttle_neutral`. With the stick there, the throttle semantic value is 0 and `safe=1` once the other sticks are released.
- Press each candidate switch and record the button id. No id is preassigned.
- Unplug USB. Within `usb_timeout_s` the console shows disconnected and `valid=0`. Plug it back. Values return only after a fresh read. On the ground the motors stay off.

### 4. Attitude, small inputs, after mapping and with diagnostics off

- Arm, do not take off. Props turn at idle and the gear stays down.
- Take off. The climb is smooth and ends in `POSITION_HOLD` near `takeoff_height` above the ground.
- A small right roll moves right. A small forward pitch moves forward. Yaw right rotates clockwise.
- Release the right stick. Tilt returns to level without a step in the motor trace. Speed falls. State becomes `POSITION_HOLD` and X/Y stop changing.
- From a level hover, a small disturbance (pause the sim and nudge, or a brief stick tap) is rejected.

### 5. Altitude

- Throttle above the detent climbs, and the climb rate stays near or under `max_ascent_speed`.
- Throttle below the detent descends, no faster than `max_descent_speed`, and slower again inside `landing_soft_height`.
- Return the throttle to the detent. Height stops changing and stays near the value at the moment it entered the deadzone.

### 6. Position and the 5 m/s cap

- Full forward stick accelerates, then horizontal speed stays at or under 5 m/s if the stick is held.
- The same cap holds on a diagonal, not only on pure pitch.
- Release. The aircraft brakes, then holds the new position rather than springing back to the old one.
- During a yaw, the held X/Y does not walk away.

### 7. Landing

- Before any arm command the state is `DISARMED` or `DIAGNOSTIC` and the motors are 0. Already seen for `DIAGNOSTIC`.
- Arm does not happen by itself.
- A normal landing with the throttle down keeps attitude and position control until contact.
- Contact, not a target altitude, is what prints the landing confirmation when supervisor mode is on.
- After `LANDED`, motors reach 0 and a USB reconnect does not spin them. A second flight needs a new arm press.

### 8. Failsafe

- Unplug USB in a hover. Stick values go to zero immediately and the state becomes `FAILSAFE`. The aircraft does not keep the last tilt.
- It stays attitude-stable if the inertial unit is alive, and it descends.
- Plug USB back in during the descent with the sticks parked in the deadzones. After `handover_valid_s` the state leaves `FAILSAFE` without a motor spike.
- Plug it back in with the throttle still in a climb. The descent continues.
- Unplug after `LANDED`. Motors stay at 0.
- If you can disable GPS in the scene tree while the USB link is fine, position hold drops out and the console says so, and the motors stay under attitude control. That must not look identical to the USB failsafe.

## Known limits

- No TX12 was connected during the batch run. Axis indices are intentionally unset.
- Hover collective and the attitude gains are the stock Webots sample, rearranged into a cascade. They have not been re-tuned in a flight.
- Vertical speeds are placeholders.
- The accelerometer is displayed and not fused. At rest, +Z was about +9.8 m/s².
- Compass yaw is not blended into the inertial yaw. A convention offset would otherwise pull the heading.
- Without `supervisor TRUE`, touchdown is a heuristic and is labeled as such in the safety module.
- `motor_command_limit` is 160 rad/s on purpose. The model can take 576. Raise it only after the sign checks.
- The Webots joystick API cannot tell you the size of the button set, and it cannot see a second controller while another program owns the first one.
- Open-loop descent after a total attitude loss is not a safe landing. The motors stay on only so the aircraft is not deliberately shut down in the air.
