# Test Day — robot bring-up, drivebase to auto-aim

Everything is behind **one** Driver Station entry: **`Robot Test`** (group `Test`).
Dpad up/down to move, right bumper to select, left bumper to go back. Pick a test
*before* pressing play. To change tests, stop and re-init.

**Work top to bottom and do not skip.** Each stage assumes the ones above it
passed. That is the whole point: when a shot misses you want it to be one of two
possible causes, not eight. Half an hour spent proving the boring stages saves
the afternoon.

Each test brings up **only** the hardware it needs, so an unplugged hood servo
can't stop you testing the drivebase.

---

## Before you start

| | |
|---|---|
| **Battery** | Fully charged, and swap at 12.0 V. A sagging battery makes the flywheel miss its RPM, the turret stall, and the hub brown-out and reset — three symptoms that all look like code bugs. Check this first every time something starts behaving oddly. |
| **Robot config** | Must be the one with all 15 devices. Test 0 tells you. |
| **Panels** | Connect to the robot's WiFi and open `192.168.43.1:8001`. Not required — every test is usable from the DS alone — but you get live graphs and can edit gains without a rebuild. |
| **Space** | Tests 1b and 6b drive several feet. Test 1a and 3a want the robot on blocks. |
| **Have ready** | Tape measure, 5 mm hex, a couple of game balls, a printed goal AprilTag (20 and 24) if you don't have a real goal. |

**Write good numbers back into the source before you go home.** Panels edits live
in RAM. Power-cycle the hub and every value you found is gone.

---

## 0 — Hardware Scan

**Run:** `0 - Hardware Scan` · nothing moves, safe on a cart.

Looks up all 15 devices and reports what's missing. Ten seconds, every session.

**Pass:** `ALL DEVICES PRESENT`.

A device missing from the config makes `Hardware.init()` throw, and the DS shows
a stack trace for OpModes that have nothing to do with the missing part. This
test turns twenty minutes of "why won't anything run" into one line.

| Symptom | Cause | Fix |
|---|---|---|
| A name shows `??` | Name mismatch or wrong device type in the config | Names are **case-sensitive**. `frontleftDrive` ≠ `frontLeftDrive`. Check the type too — the turret is a **CRServo**, not a Servo; `turretEncoder` is an **Analog Input**, not a motor port |
| `navx` missing | Not configured, or on the wrong I2C bus | Add as `NavX Micro Navigation Sensor` on the I2C port it's actually plugged into |
| `limelight` missing | Not configured | Add as `Limelight3A` on USB. It also needs its own power |

---

## 1 — Drivebase

### 1a — Motor Check

**Run:** `1 - Drivebase → 1a Motor Check` · **robot on blocks.**
Bumpers pick a motor · right trigger forward · left trigger reverse ·
dpad up/down power · **A** run-mode toggle · **Y** zero encoders.

One motor at a time, because with four running you can't tell which is wrong.

**Pass, for each of the four:**
1. The wheel that moves is the one named on screen.
2. It spins **forward** on right trigger (top of the wheel travels toward the front).
3. Its **ticks change** while it spins.
4. Current is in the same ballpark as the other three.

| Symptom | Cause | Fix |
|---|---|---|
| Wrong wheel moves | Two cables swapped, or config names on the wrong ports | Swap in the config, not in code |
| One wheel spins backwards | Motor direction | Flip that side's `LEFT_DIRECTION` / `RIGHT_DIRECTION` in `Constants.Drive` |
| **Ticks stay at 0** | **Encoder cable not plugged in** | Plug it in. If you can't, set `Constants.Drive.RUN_MODE = RUN_WITHOUT_ENCODER` — see below |
| One motor draws 3–4× the others | Something binding | Mechanical. Find it now, before it cooks |
| Motor lurches to full power and won't modulate | No encoder, in `RUN_USING_ENCODER` | Same as above |

> **Why the missing encoder matters so much.** In `RUN_USING_ENCODER` the hub runs
> its own velocity loop. A motor with no encoder reads zero velocity forever, so
> the loop concludes it isn't moving and commands full power. One unplugged cable
> makes the robot veer hard to one side — and it looks exactly like a mecanum
> roller problem, so that's where people go looking. Press **A** to switch to
> `RUN_WITHOUT_ENCODER`: if the veer disappears, you've found it.

### 1b — Drive + Heading

**Run:** `1 - Drivebase → 1b Drive + Heading` · starts at 25% speed.
Left stick translate · right stick X turn · **Y** field/robot centric ·
**dpad up** zero heading · **X** flip heading sign · right bumper for full speed.

Two separate proofs. Don't mix them.

**Robot-centric first** (mecanum mixing and directions):
- Stick forward → robot goes straight forward.
- Stick left → strafes left, **without rotating**.
- Right stick → spins in place.

**Then field-centric** (the navX):
- Press dpad up to zero. Drive forward — note the direction across the floor.
- Rotate the robot 90° **by hand**, push the stick forward again.
- It must travel the **same way across the field**.

| Symptom | Cause | Fix |
|---|---|---|
| Forward is fine, strafing crabs diagonally | Mecanum rollers mounted wrong | Looking down at the robot, the four rollers form an **X**. If they're parallel, swap a pair of wheels |
| Robot rotates when you ask it to strafe | One motor backwards or one wheel not driving | Back to 1a |
| Field-centric drives at a mirrored angle | Heading sign inverted | Press **X**, re-zero, retry. Then set `NavXIMU.INVERT = true` in the source |
| Heading drifts while sitting still | navX still calibrating, or vibration | Don't move the robot during init. A few deg/min is normal; re-zero with dpad up |
| Heading jumps around wildly | I2C noise or a loose cable | Reseat. Keep the navX cable away from motor leads |

> Rotating **counter-clockwise** must make the heading number go **up**. That's
> the convention everything downstream assumes.

---

## 2 — Ball Path

### 2a — Intake + Indexer

**Run:** `2 - Ball Path → 2a Intake + Indexer` · nothing latches, hold a button.
**RB** intake in · **RT** intake out · **A** indexer feed · **B** indexer reverse.

**Pass:** with a ball in hand — `INTAKING` pulls it **in**, `FEEDING` moves it
**toward the flywheel**, current settles at a low steady value.

| Symptom | Cause | Fix |
|---|---|---|
| Intake spits instead of sucking | Direction | Flip `Constants.Intake.DIRECTION`. **Not** the buttons — autonomous calls the same enum and would run backwards |
| Indexer feeds the wrong way | Direction | Flip `Constants.Indexer.DIRECTION` |
| Current climbs and stays high | Binding | Mechanical |

> Jam a ball deliberately for a second and note the current. That's what a stall
> looks like on this robot — it's the number you'll be trying to recognise later
> when the shooter mysteriously stops feeding.

### 2b — Stopper Gate

**Run:** `2 - Ball Path → 2b Stopper Gate`
**A** open · **B** block · bumpers ±0.01 on whichever position you're sitting at ·
dpad up/down ±0.05 s on the travel time.

Both positions in `Constants.Stopper` are guesses. Find the real ones with a ball
actually in the robot.

**Pass:**
- **BLOCKING** — smallest movement that reliably holds a ball off the flywheel.
- **OPEN** — fully clear. Watch the *ball*, not the servo horn.
- **TRAVEL_TIME_S** — flip A/B and count. This is a plain timer, not feedback:
  too short rams a ball into a half-open gate, too long adds dead time to every shot.

| Symptom | Cause | Fix |
|---|---|---|
| Servo buzzes at one end | Stalling against a stop | Back the position off until it stops buzzing |
| Ball squeezes past when blocking | Not far enough | Nudge until it holds |
| Ball catches on the way out | Gate only mostly clears | Widen OPEN. This passes on the bench and jams at match feed rate |

---

## 3 — Shooter

### 3a — Flywheel RPM

**Run:** `3 - Shooter → 3a Flywheel RPM` · **guard on, nothing loose near the wheel.**
**A** spin up · **B** stop · bumpers ±100 rpm · dpad ±500 rpm.

**Check the two motors agree before tuning anything.** The screen shows both
velocities and both currents.

**Pass:** both motors report **the same sign** and **similar magnitude**. The test
warns you if not.

> If the reversal in `Constants.Flywheel.RIGHT_DIRECTION` is wrong, the two motors
> fight each other. The wheel barely turns, both currents spike, and the PID —
> which only reads the *left* encoder — sees a low speed and pushes **harder**.
> That's how you burn a motor in ninety seconds.

**Then check the RPM number is real.** `TICKS_PER_REV = 28` assumes a bare motor
with no gearbox. Hand-spin the wheel one full turn from a standstill and confirm
the ticks move by 28. If your flywheel is geared, every RPM in the robot is wrong
by that ratio — and you'd spend the afternoon tuning a shooting table around a lie.

**Then tune,** in this order: kF until it settles near target on its own, then kP
for the remaining gap, then kD if it oscillates. Leave kI at 0.

**Watch the recovery, not the steady state.** The number that sets your fire rate
is how fast RPM comes back after a ball. Spin up, push a ball through by hand, and
watch the dip on the Panels graph.

| Symptom | Cause | Fix |
|---|---|---|
| Motors disagree / huge current | Wrong reversal | Flip `RIGHT_DIRECTION` |
| RPM reads 0 while clearly spinning | No encoder on `flywheelLeft` | Plug it in — the whole velocity loop depends on it |
| RPM reads ~4× or ~¼ of plausible | Wrong `TICKS_PER_REV` | Measure by hand |
| Never reaches target | kF too low, or battery sagging | Raise kF; check battery |
| Oscillates around target | kP too high | Lower kP, add kD |
| Takes forever to recover after a ball | Physics, not code | Lower the target RPM, or accept a slower fire rate |

### 3b — Hood Angle

**Run:** `3 - Shooter → 3b Hood Angle` · **hand on the stop button.**
Bumpers ±0.01 · dpad up/down ±0.05 · **X** near · **B** far · **Y** stow ·
**A** store as MIN · **dpad right** store as MAX.

**Find the mechanical limits first, carefully.** Start at the stowed default and
nudge outward **one 0.01 step at a time**, listening. The moment the servo buzzes
or the linkage stops moving, you've gone one step too far — back off two and store
that as the limit. Do both ends, then copy MIN/MAX into `Constants.Hood`.

> A positional servo asked past its stop doesn't give up. It holds full torque
> until something strips. Servos are the part most likely to die quietly on a test
> day. Once MIN/MAX are right they protect you from every bad preset and every
> stale auto-range value for the rest of the season.

Shot angles come later, in 6a, once you're shooting at a real goal.

> **Note:** `Hardware.initHood()` commands `DEFAULT_POSITION` (0.15) the instant
> you press INIT, before you can jog anything. Check by hand that the linkage can
> reach 0.15 before running this the first time.

---

### 3c — Ball Path + Shooter, together

**Run:** `3 - Shooter → 3c Ball Path + Shooter` · **guard on.**
**A** toggles the flywheel · right stick up/down ramps the target RPM ·
**RB** intake in · **RT** intake out · **LB** indexer feed · **LT** indexer reverse ·
dpad left/right hood ∓0.01 · dpad up/down hood ±0.05 · **X** near · **B** far ·
**Y** stow. Only the flywheel latches.

Intake, indexer, both flywheels and the hood in one OpMode. 2a/3a/3b each proved
one mechanism alone, which finds a miswired motor but tells you nothing about four
motors pulling at once. **Watch the battery line as much as the mechanisms** — the
test prints the lowest voltage seen since INIT, because the sag that matters
happens during a half-second spin-up and is easy to miss live.

What to prove:

- **Both flywheel motors agree** — same sign, similar magnitude. The PID only
  reads the left encoder, so a wrong `RIGHT_DIRECTION` shows up as the pair
  fighting: low speed, both currents high, loop pushing harder.
- **The RPM holds with the ball path running.** Spin up, then run intake and
  indexer together. A target the wheel held alone but loses under load is a power
  problem, not a PID one.
- **Voltage stays above 11 V.** Below that, any shooting table you tune now won't
  hold at the end of a match.
- **The hood stays put under vibration.** Jog it while the wheel is at speed.

> **The stopper and turret are not powered here.** Nothing initializes or commands
> them, so this test runs fine with either unplugged or half-built — which is the
> point. But that also means an installed gate sits wherever it was left with no
> holding torque, so feeding the indexer may push a ball straight through or jam
> against a dead gate. Both are expected. Gate timing is 2b; the full
> aim-rev-fire chain is 6a.

---

## 4 — Turret

### 4a — Encoder + Manual + Park (no camera)

**Run:** `4 - Turret → 4a Encoder + Manual + Park` · **start with the turret near centre.**
dpad left/right nudge · **A** park · **B** stop · **X** capture ORIGIN_DEG ·
**Y** flip INVERT_RETURN · bumpers change nudge power.
Readouts are live during **INIT**, so you can measure before anything can move.

This is the one subsystem that can destroy itself. It's a continuous-rotation
servo with wires running to it and no mechanical idea of where it is. Everything
stopping it twisting its own loom off is software, and all of that software rests
on one analog voltage.

**Four things, in order:**

**1. Is the feedback wire alive?** Nudge and watch the voltage. It must sweep
smoothly and span most of 0–3.3 V over a full turn.

> If it reads 0 or never moves, **every safety in the turret is inert**. The
> continuous angle never changes, travel stays at 0, the limit never trips, and
> the turret will happily wind until something tears. The test says
> `FEEDBACK WIRE LOOKS DEAD`. Stop and fix the wire — do not continue to test 5.

**2. Where is straight ahead?** Push the turret to dead centre by hand, press **X**.
Copy the captured value into `Constants.Turret.ORIGIN_DEG`.

**3. Does parking go the right way?** Nudge well off centre, press **A**. It must
drive **back toward** the origin and stop. If it accelerates away and pins itself,
press **Y** and retry.

> Get this right here. The same sign tells the travel limit which way the turret
> is winding, so a wrong `INVERT_RETURN` silently disables the limit in the
> direction that matters.

**4. Does the travel limit hold?** Nudge past ±180°. The nudge must **stop working
in that direction** while still working back the other way.

| Symptom | Cause | Fix |
|---|---|---|
| Voltage flat / zero | Axon 4th wire not connected | Fix it. Nothing below works without it |
| Voltage jumps erratically | Bad ground or a long unshielded run | Reseat; keep it away from motor leads |
| Park runs away from the origin | `INVERT_RETURN` wrong | Press **Y** |
| Park stops just short and sits | Below the stiction floor | Raise `MIN_AIM_POWER` or widen `RETURN_TOLERANCE_DEG` |
| `GAVE UP — turret stopped moving` | Blocked, or too little power for the load | Find the snag. Press **A** to retry. **This is a safety, not a bug** — it cuts power rather than grinding the servo |
| Travel reads nonsense at startup | Turret was wound when the robot powered on | Centre it by hand and re-init. The code can't see winding that happened while it was off |
| Turret creeps with the gains zeroed | *(fixed)* the stiction floor used to take its sign from a `+0.0` output | — |

---

## 5 — Vision and Auto-Aim

### 5a — Limelight + Distance Calibration

**Run:** `5 - Vision + Auto-Aim → 5a Limelight` · nothing moves.
**X** blue goal (20) · **B** red goal (24) · bumpers ±0.5° camera tilt ·
dpad up/down ±1 cm camera height.

**Seeing the tag.** The turret only tracks the one id for your alliance, so "the
Limelight sees tags" isn't "the robot has a target". The test lists every id in
frame, which separates the three failures cleanly.

**Calibrating the distance — do not skip this.**

> Distance is **not measured**. It's computed from the tag's vertical angle and
> three numbers that are currently guesses: camera height, tag height, camera
> tilt. Those three feed **both** the auto-ranged RPM **and** the auto-ranged hood
> angle. Get them wrong and every ranged shot misses by a consistent amount —
> which looks exactly like a badly tuned shooting table. You can lose a whole
> afternoon tuning RPM to compensate for a camera angle that's out by five
> degrees. **Fix the geometry first, then tune the table.**

1. Measure the real camera lens height and the real tag centre height.
2. Park at a tape-measured distance. Compare against the reported figure.
3. Nudge the tilt with the bumpers until they match.
4. **Re-check at a much longer distance.** Tilt errors barely show up close and
   blow up far away — one calibration point proves nothing.
5. Copy all three into `Constants.Vision`.

| Symptom | Cause | Fix |
|---|---|---|
| No ids at all | Wrong pipeline, bad exposure, not streaming | Open the Limelight web UI. `Constants.Vision.PIPELINE` must match the AprilTag pipeline index |
| Ids listed but not yours | Pointed at an obelisk tag (21/22/23) or the other goal | Not a code problem |
| Your id flickers in and out | Too far, too steep, or motion blur | Note the range where it goes unreliable — that's the real limit of auto-aim |
| Distance reads −1 with a lock | `tilt + ty ≤ 0` — geometry degenerate | Tilt is too low for the range. Raise `CAMERA_MOUNT_ANGLE_DEG` or physically angle the camera up |
| Distance right up close, wrong far away | Tilt is off | Classic. Calibrate at the far point |

### 5b — Turret Auto-Aim

**Run:** `5 - Vision + Auto-Aim → 5b Turret Auto-Aim` · **hand on STOP.**
**X** blue · **B** red · **Y** flip INVERT_OUTPUT · **A** park · bumpers nudge kP.

Only run this once 4a and 5a both pass.

**Get the direction right on the first press.** Show the tag off to one side — the
turret must swing **toward** it. If it runs the other way, press **Y** immediately.

> This is a **different flag** from the one in 4a. That one was the servo's
> gearing; this one is how the camera is mounted. Getting one right tells you
> nothing about the other.

**Then tune out the hunting.** Raise kP until it arrives briskly, add kD until it
stops overshooting, leave kI at 0.

**Then watch the wind.** Carry the tag in a circle around the robot. Travel climbs;
at the limit it should **unwrap** — swing a full turn the other way to the same
heading — and come out still pointed at the tag. Confirm it never reads LOCKED
mid-swing.

| Symptom | Cause | Fix |
|---|---|---|
| Runs away from the tag, hits the limit | `INVERT_OUTPUT` wrong | Press **Y** |
| Overshoots and oscillates | kP too high | Lower kP, add kD |
| Buzzes at centre, never settles | `MIN_AIM_POWER` kicks it further than `AIM_TOLERANCE_DEG` is wide, so it can't land in the band | Widen `AIM_TOLERANCE_DEG` or lower `MIN_AIM_POWER`. **Not** a kP problem |
| Slow and never quite arrives | kP too low, or friction | Raise kP; check the turret turns freely by hand |
| Unwrap starts and never finishes | Mechanical range is less than a full turn | The turret gives up after 0.75 s of no progress and reports `BLOCKED`. Lower `MAX_TRAVEL_DEG` so it never needs the full swing |
| Unwraps back and forth repeatedly | *(fixed)* the hysteresis guard used to be cleared mid-swing | — |

---

## 6 — Whole Robot

### 6a — Shooter End-to-End (no drive)

**Run:** `6 - Whole Robot → 6a Shooter End-to-End`
**LB** rev · **LT** fire · **RB** intake · **A** indexer · **B** reverse ·
**X**/**Y** alliance · dpad left/right hood · dpad up back to auto hood.

The same chain as the match TeleOp — same subsystems, same scheduler, same READY
gate — with the drivebase left out so the robot stays put. Instead of one status
line it breaks the gate into its five conditions and shows which are met.

**Pass:** all five green, LT fires, balls go where you point.

```
  OK    camera sees the goal
  OK    distance is valid
  OK    turret locked on
  no    flywheel at speed     <- this is what you're waiting for
  OK    gate finished opening
```

**Then build the shooting table.** From a measured distance, adjust RPM and hood
until it scores, and write that pair into the `RANGE_*` arrays in
`Constants.Flywheel` and `Constants.Hood`. Do three or four distances across your
real shooting range. Values in between are interpolated and outside are clamped,
so the near and far ends of the table matter most.

| Symptom | Cause | Fix |
|---|---|---|
| Never goes READY, camera line red | No lock | Back to 5a |
| Never READY, flywheel line red | Can't hold RPM | Back to 3a. Check the battery |
| Never READY, turret line red | Won't lock | Back to 5b |
| Fires, but slowly, one ball at a time | *(fixed)* the gate used to slam shut on every RPM dip and pay its travel time again. Now a burst is armed by the tight READY band and sustained on wider ones | If still slow, raise `RPM_KEEP_TOLERANCE` |
| Shots consistently long or short | Table wrong — **or the distance estimate is** | If it's wrong by a consistent *ratio*, suspect 5a, not the table |
| Shots scatter at the same distance | Inconsistent ball contact, or RPM not settled | Widen the wait; check compression and the hood's repeatability |
| Ball fires before the gate is open | `TRAVEL_TIME_S` too short | Raise it in 2b |

### 6b — Drive To Pose

**Run:** `6 - Whole Robot → 6b Drive To Pose` · **clear the floor.**
Hold **A** to drive (release to stop) · **B** stop · **X** re-zero pose ·
dpad ±6" target · bumpers ±45° target heading.

**Check localization before touching a gain.** Press nothing. Push the robot
forward a foot by hand: X rises by ~12. Push it left: Y rises. Rotate CCW: heading
rises.

> A bad pose looks exactly like bad tuning — the robot creeps, or oscillates, or
> confidently parks in the wrong place.

**You have no odometry pods,** so `pedroPathing/Constants.USE_ODOMETRY_PODS` is
`false` and pose comes from the drive encoders. Wheel slip goes straight into the
estimate and heading drifts. Fine for proving the controller; **not** good enough
for a real autonomous. Flip the flag the day the pods go on.

Tune one axis at a time with the target set so only that axis moves. kP until it
arrives briskly, kD to stop the overshoot, kI stays 0. Strafe usually wants more
than forward.

| Symptom | Cause | Fix |
|---|---|---|
| Robot spins on the spot instead of settling | *(fixed)* heading was sign-inverted against the drivebase | If it still happens, set `Constants.DriveToPose.INVERT_HEADING = true` |
| Drives away from the target | An axis inverted | `INVERT_FORWARD` / `INVERT_STRAFE` |
| Reported distance is way off actual | `forwardTicksToInches` untuned | Run PedroPathing's Forward/Lateral tuners (`Tuning` → Localization) |
| Pose drifts badly while driving | Drive-encoder localization | Expected without pods |

### 6c — The real TeleOp

`TeleOp 67 — RED` / `TeleOp 67 — BLUE`. Full match rehearsal: drive, intake,
aim, shoot, park the turret with dpad down. Alliance is chosen by which OpMode
you pick, so you can't start aimed at the wrong goal.

---

## Quick symptom index

| It's doing this | Look at |
|---|---|
| Nothing runs, stack trace on init | Test 0 |
| Robot veers hard one way | 1a — unplugged drive encoder |
| Strafing crabs diagonally | 1b — roller X pattern |
| Field-centric mirrored | 1b — press X |
| Motor screams / huge current | 3a — flywheel reversal |
| RPM number implausible | 3a — `TICKS_PER_REV` |
| Servo buzzing | Past its stop — 2b / 3b |
| Turret winds up its wiring | 4a — dead feedback wire |
| Turret runs away | 4a `INVERT_RETURN` / 5b `INVERT_OUTPUT` |
| Turret hunts at centre | 5b — `MIN_AIM_POWER` vs `AIM_TOLERANCE_DEG` |
| "no goal in view" | 5a — pipeline / tag id |
| Every shot off by the same amount | 5a — camera geometry, **not** the table |
| Robot spins in drive-to-pose | 6b — heading sign |
| Anything weird and intermittent | **Battery.** Always the battery |

---

## What changed for this test day

New:
- `RobotTest` — all 12 tests behind one DS entry, each initializing only what it needs.
- `Hardware.scan()` and per-group `initX()` methods.
- `VisionTuning` — camera geometry is now live-editable.
- `TurretTuning` — both INVERT flags, the aim bands, and the unwind settings moved
  here so they can be flipped at the field instead of needing a rebuild.

Fixed:
- **Turret unwrap/park could hold full power against an obstruction forever.** No
  timeout, no stall check. The robot could never shoot again for the rest of the
  match. Now gives up after 0.75 s of no progress, cuts power, and reports it.
- **Drive-to-pose heading was sign-inverted** against `Drivebase.drive()` — the
  error grew instead of shrinking and the robot spun on the spot.
- **The unwrap hysteresis guard was dead code**, cleared mid-swing every time. A
  goal near the boundary could ping-pong through 360° swings indefinitely.
- **The fire gate chattered.** A 75 RPM band on a wheel that dips ~200 per ball
  meant the stopper slammed shut and restarted its travel timer between every
  shot. Bursts now arm on the tight band and sustain on wider ones.
- **The stiction floor took its sign from a `+0.0` output**, so zeroing kP while
  tuning walked the turret off to its stop.
- **The PID integral accumulated even with kI = 0**, so typing a gain into Panels
  applied a huge stored value in one frame.
- Drive run mode is now one constant (`Constants.Drive.RUN_MODE`) you can flip.
- `PedroPathing` no longer crashes with no odometry pods.
- Telemetry no longer shows a stale distance next to a fresh "target visible".

Removed: the five standalone `*TuningOpMode` DS entries, folded into `Robot Test`.
DS list is 9 entries down to 5.

**Untested by me — all of this is compile-verified only.** Nothing here has run on
a robot. Treat every "should" as a hypothesis and keep a hand on the stop button,
especially for tests 3a, 4a, and 5b.
