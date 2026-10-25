# VEX AI (Override) code: how it fits into JAR

This branch adds the VEX AI layer to the JAR template. JAR still does all the
driving (PID, odometry, `drive_to_point`). The AI layer decides *where* to drive
and *what* to do, and talks to the Jetson and the other robot.

The Jetson side is VEX's official code from
[VEX-Robotics/VAIC_26_27](https://github.com/VEX-Robotics/VAIC_26_27)
(`JetsonExample/override.py`, unchanged). Its model detects five classes:
RedBluePin, BlueYellowPin, RedYellowPin, YellowPin, Cup.

## Data flow

```
Jetson (override.py) --USB serial ~15 Hz--> ai::jetson ----+
GPS sensor ------------------------------> PoseEstimator  |
                                              |           v
                                         JAR odom <-- WorldModel <--> TeamLink <--VEXlink--> other robot
                                              |           |
                                              v           v
                                   motion (abortable) <-- Planner -> Task -> actions -> mech (stubs)
```

One background task (`ai_task` in `src/ai/ai_main.cpp`, 50 Hz) runs pose
fusion, the motion supervisor, Jetson requests, the world model, VEXlink and
logging. Isolation routines and AI behaviors run in the competition task and
block; the background task can interrupt their moves.

## Files

| File | What it does |
|---|---|
| `include/robot-id.h` | **Set before downloading:** which robot (24" = `ROBOT_BIG`, VEXlink manager; 18" = `ROBOT_SMALL`, worker) and the VEXlink name |
| `include/robot-config.h`, `src/robot-config.cpp` | Devices. GPS and VEXlink radio ports, GPS offsets |
| `src/main.cpp` | Isolation selector, period routing, status screen, test match button |
| `src/autons.cpp` | JAR tests + Isolation routine templates (`iso_score_preload`, `iso_midfield`) |
| `include/ai/field.h` | Field frame, VEX AI goal/toggle/loader positions, quadrants, point values |
| `src/ai/pose.cpp` | GPS + odom complementary filter that corrects JAR's odom in place |
| `src/ai/world_model.cpp` | Object tracks with confidence, toggle and goal beliefs, partner info, score estimate |
| `src/ai/team_link.cpp` | VEXlink protocol (STATE 10 Hz, FIELD ~3 Hz, EVENTs sent 3x) |
| `src/ai/motion.cpp` | JAR moves that stop on deadline or when stuck |
| `src/ai/strategy.cpp` | Planner: best expected margin per second; endgame parking |
| `src/ai/actions.cpp` | Behaviors for each task, and **mechanism stubs** (`ai::mech`) |
| `src/ai/logger.cpp` | CSV logs on the Brain's SD card (`ai_000.csv`, ...) |
| `src/ai/ai_jetson.cpp` | VEX's Jetson serial class, unchanged |
| `tests/host/` | Unit tests that run this code on a laptop (`sh tests/host/run_tests.sh`) |

Changes inside JAR are small and marked "Added for VEX AI": a `motion_abort`
check in every motion loop, `set_coordinates()` no longer starts a second odom
task, and `correct_position()` for GPS fusion.

## Conventions

- **Frame:** field-centered inches, +Y and heading 0 along the GPS field
  code's +Y, heading clockwise. Same as JAR, so once odom is seeded from the GPS,
  `chassis.drive_to_point(x, y)` takes field coordinates.
- **Red side** is the left + bottom quadrants (`x + y < 0`). Alliance is
  detected from the GPS at the start of Isolation (or forced on the screen).
- **Use `ai::go_to`, `ai::turn_to`, `ai::approach`, ...** in Isolation routines
  and behaviors, not `chassis.*` directly. Only the `ai::` versions respect the
  end-of-period deadline and stuck detection.
- **No `printf`.** USB belongs to the Jetson, and `printf` output goes to it.
  Use `ai::log::event(...)`.
- VEXcode tasks are cooperative. Never call `wait()` while holding a mutex:
  the field kills the competition task at the end of a period, and a killed
  task that holds a lock freezes everything else.

## Match flow

1. **Program start / `pre_auton`:** calibrate IMU + GPS (keep the robot still
   ~3 s), seed odom from the GPS, start the background task.
2. **First autonomous call = Isolation (15 s):** set alliance, run the selected
   Isolation routine with a deadline 0.1 s before the end.
3. **Second autonomous call, or driver control while connected to a field =
   Interaction (1:45):** loop `planner.choose()` -> `run_task()`. Every non-park
   task gets a deadline that leaves time to park. In the last seconds both
   robots park in the Midfield.
4. If a match is replayed, **restart the program** (the Isolation/Interaction
   flag only resets on restart).

Not connected to a field? The controller drives normally, and the **TEST MATCH**
button on the Brain screen runs a full 15 s + 1:45 practice match (controller B
stops it).

## Setup checklist

- [ ] `robot-id.h`: put your team number in `TEAM_LINK_NAME`; set `ROBOT_ID` per robot.
- [ ] Second V5 radio on `LINK_RADIO_PORT` (VEXlink), GPS on `GPS_PORT`, both robots.
- [ ] GPS plugged into the Brain **and** the Jetson (USB-C).
- [ ] Measure GPS offsets; enter them in `robot-config.cpp` **and** the Jetson web dashboard.
- [ ] Camera offset in the Jetson dashboard; `world.cam_fwd_in` / `cam_right_in` to match.
- [ ] Drive motor cartridges (`ratio6_1` assumed) and the zero-tracker center distance in `main.cpp` (should be positive, about half the track width).
- [ ] Run `JAR: odom_test`, push the robot around, confirm odom.
- [ ] On a real field, confirm the status screen's GPS pose and "side" are right
      from each starting spot, then check the goal/toggle/loader positions in
      `field.h` by driving the robot to them.
- [ ] SD card in each Brain for logs.

## Not done yet (on purpose)

- **Mechanisms.** `ai::mech` is a stub. With `mech::simulate = true` (default)
  the robot drives the whole plan and pretends intake/scoring worked, which is
  enough to test navigation, the planner, VEXlink and logging now.
- **Opponent robots.** The official model doesn't detect robots. Collisions are
  handled only by stuck detection + `escape()`. A robot class on the Jetson
  would let the planner avoid them.
- **Toggle state.** Known only from our own and the partner's actions. A camera
  check (color in the toggle's image region) would fix that.
- **Goal contents.** Counted from scoring events, not seen. Stacked detections
  (`Track::on_goal`) could be used to check them.
- **Starting object layout.** Fill a table from Appendix A to seed the world model.
- **Values marked `TODO verify`** in `field.h`: Midfield size, blue loaders, point values.
- **Planner timings** (`drive_speed_ips`, `pickup_s`, ...) are guesses until timed on the robot.

## Adding data from the Jetson

Don't change `AI_RECORD`; VEX's dashboard and future updates depend on it.
Add a second packet type on the Jetson (`V5SerialPacket(type=2, ...)`) and a
`case` for it in `ai::jetson::parse()` (`kStateGoodPacket`). Same idea for
VEXlink: add a packet type in `team_link.h`, keep it small (about 520 B/s total),
and update the `static_assert`s.
