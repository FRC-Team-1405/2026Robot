# Shooter / Indexer / Hopper — Code Review, Bug Report & PID Tuning Guide

> **Robot**: 4-wide ball shooter · 3× Kraken X60 (shooter) · 1× Kraken X60 (indexer) · 1× Kraken X60 (hopper)
> **Gear ratio**: 1.8:1 upward (motor → wheel) · **Flywheel**: 2.25″ dia, 6 lb

---

## 1 — Critical Bugs

### Bug 1: Motor 3 initialized twice, Motor 3 follower set twice (Shooter.java constructor)

```java
// Lines 100-106
SimProfiles.initShooter(shooterMotor3);  // line 102 — first call ✓
SimProfiles.initShooter(shooterMotor3);  // line 103 — DUPLICATE (copy-paste error)

shooterMotor3.setControl(new Follower(... MotorAlignmentValue.Opposed));  // line 105 ✓
shooterMotor3.setControl(new Follower(... MotorAlignmentValue.Opposed));  // line 106 — DUPLICATE
```

**Impact**: Wasted CAN bus traffic on startup. Harmless but sloppy.
**Fix**: Delete lines 103 and 106.

---

### Bug 2: Motor 3 current draw reads Motor 2 (Shooter.java:155)

```java
double shooterMotor3CurrentDraw = shooterMotor2.getSupplyCurrent().getValueAsDouble();
//                                 ^^^^^^^^^^^ should be shooterMotor3
```

**Impact**: SmartDashboard shows Motor 2's current for Motor 3. You cannot detect Motor 3 failures or stalls.
**Fix**: Change `shooterMotor2` → `shooterMotor3`.

---

### Bug 3: `highError` / `lowError` reset to 0 every periodic cycle (Shooter.java:161-175)

```java
// These are LOCAL variables — they reset every 20ms call!
double highError = 0.0;
double lowError = 0.0;
```

**Impact**: The high/low error tracking logic does nothing. SmartDashboard always shows ~0.
**Fix**: Make them instance fields (`private double highError = 0.0;` at class level).

---

### Bug 4: `setShooterSpeed` calls `speed.get()` three times (Shooter.java:86-91)

```java
private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    double value = speed.get().in(RotationsPerSecond);          // call 1 (unused!)
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(speed.get()));  // call 2
    shooterTarget = speed.get().in(RotationsPerSecond);         // call 3
}
```

**Impact**: If the supplier is dynamic (e.g. linked to joystick), each `.get()` could return a different value. The motor gets a different setpoint than `shooterTarget`, breaking the `isReadyToFire()` settle logic.
**Fix**: Cache the result once:
```java
private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    AngularVelocity target = speed.get();
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(target));
    shooterTarget = target.in(RotationsPerSecond);
}
```

---

### Bug 5: `setShooterMotor()` is defined but never called (Shooter.java:58-84)

The TalonFX configuration method with PID values is defined but commented out in the constructor (line 108). **The real robot motors are running with factory-default PID values.**

**Impact**: All the PID tuning in `setShooterMotor()` (kP, kI, kD, kV, MotionMagic) is never applied to the hardware. The motors rely entirely on SimProfiles in simulation, and **have no control loop configured on the real robot.**
**Fix**: Call `setShooterMotor()` in the constructor (uncomment line 108).

---

### Bug 6: Indexer and Hopper have NO TalonFX configuration on real robot

Both `Indexer.java` and `Hopper.java` only call `SimProfiles.init*()` which applies PID/MotionMagic configs **only in simulation**. On the real robot, these motors run with factory defaults — no PID gains, no MotionMagic profile.

**Impact**: On the real robot, MotionMagicVelocityVoltage commands are sent to motors with no PID gains configured, causing unpredictable behavior.
**Fix**: Add `configureMotor()` methods to both subsystems (see Section 5).

---

### Bug 7: SmartDashboard `SettleCount` logged twice (Shooter.java:195,199)

```java
SmartDashboard.putNumber("Shooter/SettleCount", settleCount);  // line 195
// ... other entries ...
SmartDashboard.putNumber("Shooter/SettleCount", settleCount);  // line 199 — DUPLICATE
```

---

## 2 — Design & Logic Issues

### Issue 1: AutoFire never stops the shooter

```java
// AutoFire.java — sequence:
shooterSubsytem.runShooter(),
Commands.waitUntil(() -> shooterSubsytem.isReadyToFire()),
indexerSubsystem.runIndexer(indexerVelocity),
Commands.waitUntil(() -> !shooterSubsytem.isReadyToFire()),
indexerSubsystem.runStopIndexer(),
hopper.runStopHopper()
// ← shooter is STILL running!
```

**Fix**: Add `shooterSubsytem.stopShooter()` at the end, or make it conditional if you want the shooter to keep spinning for rapid fire.

---

### Issue 2: AutoFire fire-detection logic is fragile

The "ball has been fired" is detected by `!isReadyToFire()` — i.e., the shooter speed dropped below threshold. This works but has race conditions:
- If the shooter never fully locks (e.g., borderline speed), the sequence stalls.
- If the ball is light and barely affects speed, the shooter may never drop out of "locked."

**Consider**: Adding a timeout, or using indexer rotation count to detect a ball has been fed.

---

### Issue 3: Unused imports in Shooter.java

```java
import java.util.prefs.Preferences;          // unused (uses WPILib Preferences via Constants)
import java.lang.annotation.Target;           // unused
import java.security.spec.DSAPrivateKeySpec;  // unused (!)
```

---

### Issue 4: Hopper speed is very slow (1 RPS)

At 1 RPS the hopper is barely turning. For a ball-feeding mechanism, typical speeds are 5-15 RPS.

---

### Issue 5: Peak voltage limited to ±8V

Kraken X60 can handle 12V. Limiting to 8V reduces max available torque by 33%. This artificially caps acceleration and top speed.

**Consider**: Raising to 10-11V for the shooter, or 12V if your battery/breaker can handle it.

---

## 3 — PID Analysis (Root Cause of Slow Ramp-Up)

### Current Shooter PID Values

| Gain | Value | Comment |
|------|-------|---------|
| **kP** | **0.0** | ⚠️ **ZERO** — no proportional correction at all |
| **kI** | **0.0015** | Tiny integral — sole source of error correction |
| **kD** | **0.0** | OK for velocity loops |
| **kV** | **0.12** | ✅ Correct feedforward for Kraken X60 (~8.33 RPS/V) |
| **kS** | **0.0** | ⚠️ No static friction compensation |
| **MotionMagic Accel** | **30 RPS/s** | ~1.67s theoretical ramp to 50 RPS |

### Why It Takes 5 Seconds

The MotionMagic profile generates a velocity trajectory that ramps from 0 → 50 RPS in ~1.67s. The **kV feedforward** provides the right voltage for the *target* velocity at each step. **But there is no proportional gain to correct the error between actual and target velocity.**

The only error correction comes from **kI = 0.0015**, which accumulates at ~0.0015V per loop iteration per RPS of error. At 50 RPS error, that's only 0.075V per cycle (20ms). It takes hundreds of cycles for the integral term to build up enough correction to overcome friction, inertia, and load.

**The fix is simple: add proportional gain.**

### Recommended Shooter PID Starting Values

| Gain | Recommended | Rationale |
|------|-------------|-----------|
| **kS** | **0.15** | Overcomes static friction. Start at 0.1, increase until motor barely moves at 0 RPS target. |
| **kV** | **0.12** | ✅ Keep as-is. Correct for Kraken X60. |
| **kP** | **0.3** | 1 RPS error → 0.3V correction. This is the **critical missing piece**. |
| **kI** | **0.0** | Remove integral. With proper kP+kV, kI is unnecessary and causes windup/oscillation. |
| **kD** | **0.0** | Keep at 0 for velocity control. |
| **MotionMagic Accel** | **60** | Double acceleration for faster ramp. At 60 RPS/s, 50 RPS target in ~0.83s. |

With these values, expected ramp to 50 RPS: **< 1.5 seconds**.

---

## 4 — Tuning Guide: Shooter (Step-by-Step)

### Prerequisites
- Robot on blocks (wheels free-spinning, no ball contact)
- SmartDashboard open showing: `Shooter/ShooterMotor1RPS`, `Shooter/Error`, `Shooter/Locked`
- Start with LOW target speed (SHORT = 10 RPS)

### Step 1: Verify kV (Feedforward)

The velocity feedforward kV tells the motor how many volts are needed per RPS. For Kraken X60 ungeared:

```
kV = 1 / (free_speed_RPS / supply_voltage) = 1 / (100 / 12) ≈ 0.12
```

With a 1.8:1 gear ratio where the wheel spins faster than the motor, the motor sees less load but the same kV applies to motor-side RPS.

**Test**:
1. Set kP=0, kI=0, kD=0, kS=0, **kV=0.12**
2. Command 40 RPS
3. Motor should reach ~80-90% of target with feedforward alone
4. If it overshoots, decrease kV. If undershoots significantly, increase kV.

### Step 2: Find kS (Static Friction)

kS compensates for friction/drag that prevents the motor from moving at low voltages.

**Test**:
1. Keep kP=0, kI=0, kV=0.12
2. Command 1 RPS
3. If the motor doesn't move, increase kS by 0.05 until it barely starts spinning
4. Typical range: 0.05 – 0.25 for Kraken X60

### Step 3: Tune kP (Proportional — THE KEY STEP)

kP corrects the remaining error between target and actual velocity.

**Test**:
1. Set kV=0.12, kS=(from step 2), kI=0, kD=0
2. Start with **kP = 0.1**
3. Command 30 RPS. Observe:
   - Error should settle to near 0 within ~0.5s
   - If sluggish (error takes >1s to settle) → **increase kP by 0.1**
   - If oscillating (velocity bouncing above and below target) → **decrease kP by 0.05**
4. Repeat at 50 RPS (LONG speed) to test under full load
5. Target: steady-state error < 1 RPS, no oscillation

**Typical range for Kraken X60 shooter: kP = 0.2 – 0.8**

### Step 4: Tune MotionMagic Acceleration

This controls how fast the motor ramps to target speed.

**Test**:
1. With good kP, kV, kS from above
2. Start at **MotionMagicAcceleration = 40**
3. Command 50 RPS from standstill, time the ramp
4. Increase to 60, 80, 100 — watch for:
   - Motor following the profile cleanly (error stays small during ramp)
   - Current draw staying reasonable (< 40A per motor during ramp)
   - No belt slipping or mechanical stress sounds
5. If error spikes during ramp → acceleration is too high for your kP, either reduce accel or increase kP

### Step 5: Verify Fire Detection

1. With tuned PID, run the full AutoFire sequence
2. Confirm: shooter locks (Shooter/Locked = true), indexer feeds ball, speed dips on fire, indexer stops
3. If shooter never reaches "locked": reduce TIGHT threshold or increase STABLE_COUNT
4. If shooter doesn't drop out of "locked" on fire: increase TIGHT threshold

### Step 6: (Optional) Add kI for Steady-State Perfection

Only if, after kP tuning, there's a persistent ~1-2 RPS steady-state error:
1. Set kI = 0.001 (very small!)
2. If error goes to 0 → done
3. If oscillation appears → reduce kI or add kD = 0.001

**⚠️ In most FRC shooters, kP + kV + kS is sufficient. kI is rarely needed and often harmful.**

---

## 5 — Tuning Guide: Indexer

### The Problem

The indexer has **no TalonFX configuration on the real robot**. The `SimProfiles.initIndexer()` config only runs in simulation. You must add motor configuration to the subsystem.

### Add Configuration (Indexer.java)

Add this method and call it from the constructor:

```java
private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kS = 0.1;
    configs.Slot0.kV = 0.12;
    configs.Slot0.kP = 0.5;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;

    configs.Voltage.withPeakForwardVoltage(Volts.of(10))
                   .withPeakReverseVoltage(Volts.of(-10));

    configs.MotionMagic.MotionMagicAcceleration = 40;  // was 10 in sim — too slow

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = indexerMotor.getConfigurator().apply(configs);
        if (status.isOK()) break;
    }
    if (!status.isOK()) {
        System.out.println("Indexer config failed: " + status.toString());
    }
}
```

### Tuning Steps

1. **Increase target speed**: Change `INDEXER_VELOCITY` from 20 RPS to 30-40 RPS via Preferences
2. **Increase MotionMagic Acceleration**: 40+ RPS/s for snappy ball feeding
3. **Tune kP**: Start at 0.5, same process as shooter (step 3 above)
4. **Raise voltage limit**: 10V minimum; the indexer needs torque to grab balls

---

## 6 — Tuning Guide: Hopper

### The Problem

Same as indexer — no TalonFX configuration on real robot. Also, 1 RPS forward speed is far too slow.

### Add Configuration (Hopper.java)

Same pattern as indexer — add `configureMotor()` to constructor.

### Tuning Steps

1. **Increase speed**: Change `Hopper/Forward` from 1.0 RPS to 5-10 RPS via Preferences
2. **MotionMagic Acceleration**: 20-30 RPS/s is fine for a hopper
3. **kP**: Start at 0.3, adjust as needed

---

## 7 — Quick Reference: Recommended Starting Values

| Subsystem | kS | kV | kP | kI | kD | MM Accel | Peak V | Target Speed |
|-----------|-----|------|------|------|------|----------|--------|-------------|
| **Shooter** | 0.15 | 0.12 | 0.3 | 0.0 | 0.0 | 60 | ±10V | 10-50 RPS |
| **Indexer** | 0.1 | 0.12 | 0.5 | 0.0 | 0.0 | 40 | ±10V | 30-40 RPS |
| **Hopper** | 0.1 | 0.12 | 0.3 | 0.0 | 0.0 | 25 | ±10V | 5-10 RPS |

---

## 8 — Summary of All Fixes Needed

| # | File | Line | Severity | Description |
|---|------|------|----------|-------------|
| 1 | Shooter.java | 103 | Low | Duplicate `initShooter(shooterMotor3)` |
| 2 | Shooter.java | 106 | Low | Duplicate `shooterMotor3.setControl(Follower)` |
| 3 | Shooter.java | 155 | **High** | Motor 3 current reads Motor 2 |
| 4 | Shooter.java | 161-175 | **High** | `highError`/`lowError` are local, never accumulate |
| 5 | Shooter.java | 87-90 | Medium | `speed.get()` called 3× (possible inconsistency) |
| 6 | Shooter.java | 108 | **Critical** | `setShooterMotor()` never called — no PID on real robot |
| 7 | Shooter.java | 60 | **Critical** | kP=0.0 — no proportional gain (5s ramp root cause) |
| 8 | Shooter.java | 64 | Medium | kS=0.0 — no static friction compensation |
| 9 | Indexer.java | — | **Critical** | No TalonFX config on real robot |
| 10 | Hopper.java | — | **Critical** | No TalonFX config on real robot |
| 11 | AutoFire.java | — | Medium | Shooter never stopped after firing |
| 12 | Constants.java | 188 | Medium | Hopper speed 1 RPS too slow |
| 13 | Shooter.java | 9-12 | Low | Unused imports |
| 14 | Shooter.java | 199 | Low | Duplicate SmartDashboard key |
