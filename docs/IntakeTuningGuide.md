# Intake Subsystem Tuning & Testing Guide

This guide walks through tuning the intake from a cold start — safely, in order, with no assumptions about the current state of the robot. Follow the phases in sequence. **Never skip a phase.**

---

## Dashboard Setup

Before any tuning, add these SmartDashboard entries to your Elastic layout under an "Intake" tab. The subsystem publishes all of these automatically every loop:

| Key | What it tells you |
|-----|-------------------|
| `Intake/DeployPosition` | Current deploy motor encoder position (rotations) |
| `Intake/DeployTarget` | The position MotionMagic is driving toward |
| `Intake/DeployError` | Absolute error between current and target (rotations) |
| `Intake/DeployVelocity` | Deploy motor velocity (rotations/sec) |
| `Intake/DeployCurrent` | Deploy motor stator current (amps) |
| `Intake/PickupCurrent` | Pickup roller stator current (amps) |
| `Intake/IsDeployed` | Boolean — software thinks intake is out |
| `Intake/IsPickupActive` | Boolean — rollers are commanded |
| `Intake/AtTarget` | Boolean — settled at target position |
| `Intake/SettleCount` | How many consecutive cycles within tolerance |
| `Intake/StallCount` | Consecutive stall-detection cycles (0 = healthy) |

**Recommended tools:** Elastic Dashboard or Shuffleboard with a graph widget on `DeployPosition`, `DeployVelocity`, and `DeployCurrent`. You will be watching these graphs constantly.

---

## Safety Rules

These apply at all times during tuning:

1. **Have a driver at the e-stop at all times.** The first time you run any phase, someone's thumb is on the spacebar.
2. **Disable soft limits ONLY when establishing positions for the first time.** Re-enable them immediately after.
3. **Never tune PID gains while the intake is near a mechanical stop.** Start at center, not at extremes.
4. **If `StallCount` is climbing and you didn't expect a stop, e-stop immediately** — the motor is fighting the mechanism.
5. **Always retract the intake before transport.** Check `IsDeployed = false` before moving the robot.

---

## Phase 1 — Establish Safe Positions (No PID Yet)

**Goal:** Find the correct encoder values for IN, OUT, and CENTER before any closed-loop motion.

### 1.1 Temporarily Disable Soft Limits

In `Constants.java`, set `SOFT_LIMIT_MARGIN` to a very large number (e.g., `999.0`) so the soft limits do not block manual movement. You'll fix this at the end of this phase.

### 1.2 Back-drive the mechanism by hand

With the robot **disabled**, manually push the intake arm through its full range of motion. Watch `Intake/DeployPosition` on the dashboard. Record:

- Position at full retract (hard stop against the frame) → candidate for `Intake/In`
- Position at full deploy (hard stop against the bumper or floor) → candidate for `Intake/Out`
- Approximate center position (where you want the intake to pause mid-deploy) → candidate for `Intake/Center`

> **Note:** The motor uses Brake mode. If it resists movement while disabled, temporarily set `NeutralMode` to `Coast` in `configureIntakeMotor()`, restart, move it, record positions, then set it back to `Brake`.

### 1.3 Set the Preferences values

Open the **Elastic/Shuffleboard Preferences widget** (or use the WPILib Preferences table in NetworkTables). Update:

| Preference Key | New Value |
|----------------|-----------|
| `Intake/In` | (your retracted encoder value) |
| `Intake/Out` | (your deployed encoder value) |
| `Intake/Center` | (your center encoder value) |

**Rule of thumb:** Set `Intake/In` to be about **3–5 rotations away from the physical hard stop**, not at it. This gives the soft limit and stall detection a chance to act before you hit metal.

### 1.4 Restore soft limits

In `Constants.java`, restore `SOFT_LIMIT_MARGIN = 2.0`. Redeploy code. Verify in NetworkTables that the TalonFX's soft limit thresholds are:
- Forward limit ≈ `INTAKE_MOTOR_OUT + 2.0`
- Reverse limit ≈ `INTAKE_MOTOR_IN - 2.0`

---

## Phase 2 — Deploy Motor PID (MotionMagic Position Control)

The deploy motor uses **MotionMagic** which generates a smooth velocity profile from your current position to the target. There are two separate concerns: the **feedforward gains** (kS, kV, kG) which tell the motor what voltage to apply for a given motion, and the **feedback gains** (kP, kD) which correct for error.

Tune feedforward first. Always.

### 2.1 kS — Static Friction (Feedforward)

**What it does:** Overcomes static friction before the motor starts moving.

**Starting value:** `DEPLOY_KS = 0.25`

**Test:**
1. Enable the robot with intake at center position.
2. Command a move to OUT position.
3. Watch `DeployVelocity`. If the intake hesitates for a long time before moving, **increase kS** by 0.05 at a time.
4. If the intake creeps slightly even when you expect it to be stationary (without a setpoint), **decrease kS**.

**Healthy behavior:** Motor begins moving within one robot loop cycle (20ms) of receiving a setpoint.

---

### 2.2 kV — Velocity Feedforward

**What it does:** Sets the voltage needed to sustain a given cruise velocity. Wrong kV means MotionMagic over/undershoots the profile.

**Starting value:** `DEPLOY_KV = 0.12`

**Test:**
1. Command a long move (IN → OUT).
2. Watch `DeployVelocity` against your `DEPLOY_CRUISE_VELOCITY` (80 RPS).
3. If actual velocity never reaches the cruise target → **increase kV**.
4. If actual velocity blows past the cruise target → **decrease kV**.

**Exact formula if you have a datasheet:**  
`kV = 1 / (free_speed_RPS)` — for a Kraken X60 (~6000 RPM free = 100 RPS), theoretical kV ≈ 0.01. In practice, under load and with gearing, 0.12 is a reasonable starting point.

---

### 2.3 kG — Gravity Feedforward

**What it does:** Applies a constant voltage to hold against gravity if the arm is affected by it.

**Starting value:** `DEPLOY_KG = 0.0`

If your intake arm's weight causes it to sag or droop when the motor is in neutral, increase kG until it holds steady in the deployed position with no setpoint active. If your arm is spring-loaded or gravity-neutral, leave this at 0.

---

### 2.4 kP — Proportional Feedback

**What it does:** Drives toward the target based on position error. The primary correction gain.

**Starting value:** `DEPLOY_KP = 4.8`

**Test procedure (slow and safe):**
1. Start with kP at 0. Command a move to OUT. The arm will lag badly or not reach target — this is expected.
2. Increase kP by **0.5 at a time**, redeploying between each change.
3. Watch `DeployError`. You want error to approach zero without oscillation.

**Signs of correct kP:**
- `DeployError` approaches 0.0 smoothly
- `AtTarget = true` within 1–2 seconds of commanding a position
- No visible oscillation or buzzing

**Signs kP is too high:**
- The arm oscillates around the target (bounces back and forth)
- `StallCount` starts incrementing near the target
- Motor makes a buzzing or grinding noise while holding position

**Signs kP is too low:**
- Arm stops short of the target and `AtTarget` never goes true
- `DeployError` settles at 1–3+ rotations and stays there

---

### 2.5 kD — Derivative Feedback

**What it does:** Damps oscillation by opposing velocity. Only add this if kP alone causes oscillation.

**Starting value:** `DEPLOY_KD = 0.1`

**Test:**
1. After finding a kP that mostly works but has slight oscillation at the endpoint, add kD.
2. Increase kD by **0.01 at a time**.
3. The oscillation should reduce. Stop when the motion is critically damped (smooth approach to target, no overshoot).

**Warning:** Too much kD causes the arm to slow excessively before reaching the target and feel sluggish. Keep it small.

---

### 2.6 MotionMagic Profile (Cruise Velocity, Acceleration, Jerk)

These set the shape of the motion profile — how fast and how smoothly the arm moves.

**Current values:**
```
DEPLOY_CRUISE_VELOCITY = 80.0   // rotations/sec
DEPLOY_ACCELERATION    = 160.0  // rotations/sec²  (reaches cruise in ~0.5 sec)
DEPLOY_JERK            = 1600.0 // rotations/sec³  (S-curve smoothing)
```

**Tuning strategy — always start SLOWER than you need:**

1. **First test:** Set cruise velocity to `20.0` and acceleration to `40.0`. This is roughly 25% of the target speed. Confirm the arm moves smoothly through its full range before increasing.

2. **Increase gradually:** Double the values each test until you approach the target, or until the arm starts jerking, slipping, or tripping the current limit.

3. **Jerk:** High jerk means the S-curve is tight (more trapezoidal profile). Low jerk means more S-curve smoothing (gentler on the mechanism). `DEPLOY_JERK = 2 × DEPLOY_ACCELERATION` is a reasonable starting ratio.

4. **Watch `DeployCurrent` while moving.** If it spikes above `DEPLOY_STATOR_LIMIT` (40A) during a move, your acceleration is too aggressive. Back off acceleration by 20%.

**Target feel:** Full IN→OUT deploy should complete in under 1 second without jerking, current spikes, or noise.

---

## Phase 3 — Stall Detection Calibration

The stall detection in `periodic()` watches for: motor commanded to move + velocity near zero + current above threshold for N consecutive cycles. Getting these thresholds right is critical — too sensitive and it trips during normal moves; too loose and it won't protect the chain.

**Constants to tune:**
```java
STALL_CURRENT_THRESHOLD = 35.0   // amps
STALL_CYCLES_THRESHOLD  = 10     // cycles (~200ms at 50Hz)
```

### 3.1 Measure normal operating current

1. Run a full IN→OUT→IN cycle.
2. Watch `Intake/DeployCurrent` peak during the move. Note the peak value.
3. Also watch current when holding position — this should be **very low** (1–5A in Brake mode).

### 3.2 Measure hard stop current

With **a person ready to e-stop**:
1. Note the physical hard stop position.
2. Manually set `Intake/Out` in Preferences to 5 rotations **past** the actual hard stop.
3. Command the intake out. It will hit the stop and stall.
4. Watch the `DeployCurrent` spike. Record the peak value.
5. **E-stop immediately.** This is the value you'll set `STALL_CURRENT_THRESHOLD` to, minus a 5A margin.

> Example: If the stall current is 45A, set `STALL_CURRENT_THRESHOLD = 40.0`.

### 3.3 Set STALL_CYCLES_THRESHOLD

- `10 cycles = ~200ms` at the default 50Hz robot loop. This is a good balance — long enough to not trip on a brief current spike from acceleration, short enough to cut power before chain damage.
- If you find it triggers too often during normal fast moves: increase to 15.
- If you want faster protection: decrease to 5–7.

### 3.4 Restore correct position values

After stall testing, **immediately restore** `Intake/Out` to the correct value in Preferences.

---

## Phase 4 — Pickup Roller Tuning

The pickup motor uses **MotionMagic velocity control** (constant speed roller).

**Constants to tune:**
```java
PICKUP_KP = 0.11
PICKUP_KS = 0.25
PICKUP_KV = 0.12
PICKUP_MOTOR_IN  = 80.0   // rotations/sec inward
PICKUP_MOTOR_OUT = -25.0  // rotations/sec outward (ejection)
```

### 4.1 Verify direction

1. Command `runPickupIn()`. Confirm rollers spin in the direction that intakes game pieces.
2. If backwards, negate the `PICKUP_MOTOR_IN` value in Constants (or invert the motor in config).

### 4.2 Tune kS (static friction)

Same as deploy motor. Start at 0.25. If the roller hesitates before reaching speed, increase by 0.05 increments.

### 4.3 Tune kV and kP

1. Command `runPickupIn()` and watch `Intake/PickupCurrent`.
2. The roller should spin up quickly and hold steady speed. If it oscillates in speed, increase kP slightly.
3. Target: reach commanded velocity (80 RPS) and hold it within ±2 RPS.

### 4.4 Tune PICKUP_MOTOR_IN speed

80 RPS is the starting recommendation. Test with an actual game piece:
- **Too slow:** Piece bounces off or doesn't get grabbed.
- **Too fast:** Piece gets fired back out the other side.
- Adjust in Preferences (`Pickup/In`) without redeploying. Changes in Preferences take effect on the **next robot enable** (they are loaded at class initialization).

> **Important:** Preferences values are loaded once at startup in the static initializer of `IntakePreferences`. To test a new speed, change the value in the Preferences widget, then **disable and re-enable** (or deploy code).

---

## Phase 5 — Settle and Tolerance Tuning

These control when `AtTarget` becomes true and commands proceed.

```java
POSITION_TOLERANCE = 1.0   // rotations
SETTLE_COUNT       = 5     // consecutive cycles
```

### How it works

Each robot loop (~20ms), if `|currentPosition - target| < POSITION_TOLERANCE`, the `settleCount` increments. Once it reaches `SETTLE_COUNT`, `isAtTarget()` returns true and the waiting command continues. If the motor drifts out of tolerance at any point, `settleCount` resets to zero.

### Tuning guidelines

| Symptom | Fix |
|---------|-----|
| Commands never proceed (`AtTarget` stays false) | Increase `POSITION_TOLERANCE` or decrease `SETTLE_COUNT` |
| Commands proceed before arm is truly settled | Decrease `POSITION_TOLERANCE` or increase `SETTLE_COUNT` |
| Arm oscillates, `AtTarget` flickers on/off | Reduce kP, increase kD, or increase `POSITION_TOLERANCE` |
| Acceptable positional accuracy | `POSITION_TOLERANCE = 0.5–1.5` rotations is typical |

**Recommended final values:**
- `POSITION_TOLERANCE = 1.0` (start here, tighten once PID is stable)
- `SETTLE_COUNT = 5` (~100ms at 50Hz — enough to confirm it's truly settled)

---

## Phase 6 — Full System Integration Test

Once all individual components are tuned, run the full intake workflow:

### Test sequence

1. **Retract test:** Robot starts with intake out. Press D-pad down. Confirm: rollers stop, arm drives to IN, `IsDeployed = false`, `AtTarget = true`.

2. **Deploy test:** Press D-pad up. Confirm: arm drives to OUT, `IsDeployed = true`, `AtTarget = true`. Time the deploy — should be under 1 second.

3. **Roller + deploy test:** Deploy intake, then hold left bumper. Confirm rollers run while bumper is held, stop when released.

4. **Stall test (controlled):** With intake deployed, physically restrain the arm gently while it's moving. Confirm `StallCount` increments and motor cuts before any mechanical crunch. Check the log for the `STALL DETECTED` message.

5. **Soft limit test:** In Preferences, set `Intake/Out` to `softForward + 1` (one rotation past the soft limit). Command deploy. Confirm the motor stops at the soft limit, not past it. Restore the correct OUT value immediately.

6. **Repeated cycle test:** Run 10 full IN→OUT→IN cycles. Confirm `AtTarget` always goes true, no `StallCount` accumulation during normal moves, consistent timing.

---

## Quick Reference — Tuning Starting Values

| Constant | Value | Units | Notes |
|----------|-------|-------|-------|
| `DEPLOY_KP` | 4.8 | V/rotation | Decrease if oscillation |
| `DEPLOY_KI` | 0.0 | V/(rotation·s) | Leave at 0 |
| `DEPLOY_KD` | 0.1 | V/(rotation/s) | Increase to damp oscillation |
| `DEPLOY_KS` | 0.25 | V | Tune first |
| `DEPLOY_KV` | 0.12 | V/(rotation/s) | Match to motor free speed |
| `DEPLOY_KG` | 0.0 | V | Only if gravity-affected |
| `DEPLOY_CRUISE_VELOCITY` | 80.0 | RPS | Start at 20, increase |
| `DEPLOY_ACCELERATION` | 160.0 | RPS² | Start at 40, increase |
| `DEPLOY_JERK` | 1600.0 | RPS³ | 2× acceleration is a good ratio |
| `PICKUP_KP` | 0.11 | V/(RPS) | Increase if speed is unstable |
| `PICKUP_KS` | 0.25 | V | Tune first |
| `PICKUP_KV` | 0.12 | V/(RPS) | Match to motor free speed |
| `PICKUP_MOTOR_IN` | 80.0 | RPS | Increase if pieces aren't grabbed |
| `PICKUP_MOTOR_OUT` | -25.0 | RPS | Negative = outward |
| `STALL_CURRENT_THRESHOLD` | 35.0 | A | Measure actual stall, minus 5A |
| `STALL_CYCLES_THRESHOLD` | 10 | cycles | ~200ms at 50Hz |
| `DEPLOY_STATOR_LIMIT` | 40.0 | A | Hard cap; raise only if needed |
| `POSITION_TOLERANCE` | 1.0 | rotations | Tighten once PID is stable |
| `SETTLE_COUNT` | 5 | cycles | ~100ms; increase for slower settle |
| `SOFT_LIMIT_MARGIN` | 2.0 | rotations | Buffer beyond IN/OUT positions |

---

## Common Problems & Fixes

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| Intake doesn't move at all | kP = 0, or motor config not applied | Verify config applied in constructor; check kP > 0 |
| Arm drifts when holding position | `NeutralMode = Coast` or kP too low | Verify Brake mode in config; increase kP |
| `AtTarget` never becomes true | Tolerance too tight or PID can't reach target | Increase `POSITION_TOLERANCE`; check kP |
| Stall detection fires during normal deploy | `STALL_CURRENT_THRESHOLD` too low | Increase threshold to 5A above normal peak |
| Soft limits blocking normal motion | IN/OUT positions set past the soft limit | Adjust `Intake/In` or `Intake/Out` in Preferences to be inside the soft limits |
| Roller speed too weak to grab pieces | `PICKUP_MOTOR_IN` too low | Increase `Pickup/In` in Preferences, re-enable |
| Chain damage on hard stop | Stall detection not configured | Run Phase 3 of this guide |
| Position "forgets" after restart | ❌ Old bug (fixed) — was `stopIntake()` in `finallyDo()` | Confirm no `finallyDo` on deploy commands |
| Different speeds after code deploy | Preferences not set | Check Preferences widget — values load from there, not code defaults after first deploy |
