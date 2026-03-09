// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPhysicalProperties;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.sim.SimProfiles;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor1 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_2);
  private final TalonFX shooterMotor3 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_3);

  private final MotionMagicVelocityVoltage m_VelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final NeutralOut m_Brake = new NeutralOut();

  private LinearFilter filter = LinearFilter.movingAverage(50);

  // Rolling std dev via E[x²] - E[x]²
  private LinearFilter velocityMeanFilter = LinearFilter.movingAverage(50);
  private LinearFilter velocityMeanSqFilter = LinearFilter.movingAverage(50);

  private double highError = 0.0;
  private double lowError = 0.0;

  private int settleCount = 0;
  private int shotCount = 0;

  private double shooterTarget = 0.0;
  private double shooterStartTimestamp = 0.0;
  private double timeToLockSeconds = 0.0;

  private boolean locked = false;
  private boolean wasLocked = false;

  public boolean isReadyToFire() {
    return locked;
  }

  public void setShooterMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.3;   // Proportional: 1 RPS error → 0.3V correction
    configs.Slot0.kI = 0.0;   // No integral needed with proper kP+kV
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.12;  // Feedforward: correct for Kraken X60 (~8.33 RPS/V)
    configs.Slot0.kS = 0.15;  // Static friction compensation

    configs.Voltage.withPeakForwardVoltage(Volts.of(10)).withPeakReverseVoltage(Volts.of(-10));

    configs.MotionMagic.MotionMagicAcceleration = 60; // 60 RPS/s → ~0.83s ramp to 50 RPS

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterMotor1.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());

      shooterMotor1.setControl(m_Brake);
    }

  }

  private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    AngularVelocity target = speed.get();
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(target));
    shooterTarget = target.in(RotationsPerSecond);
    shooterStartTimestamp = Timer.getFPGATimestamp();
    timeToLockSeconds = 0.0;
  }

  private void shooterStop() {
    shooterTarget = 0.0;
    shooterMotor1.setControl(m_Brake);
  }

  /** Creates a new Shooter. */
  public Shooter() {
    SimProfiles.initShooter(shooterMotor1);
    SimProfiles.initShooter(shooterMotor2);
    SimProfiles.initShooter(shooterMotor3);
    shooterMotor2.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    shooterMotor3.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    setShooterMotor();
    SmartDashboard.putNumber("Shooter/TestTargetRPS", 10.0);
    stopShooter();
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setShooterSpeed(speed), this);
  };

  public Command runShooter() {
    return Commands.runOnce(() -> setShooterSpeed(requestedSpeed), this);
  };

  /** Reads Shooter/TestTargetRPS from SmartDashboard and spins up to that speed. */
  public Command runShooterAtTestRPS() {
    return Commands.runOnce(
        () -> setShooterSpeed(
            () -> RotationsPerSecond.of(SmartDashboard.getNumber("Shooter/TestTargetRPS", 10.0))),
        this);
  }

  private Supplier<AngularVelocity> requestedSpeed = () -> Constants.ShooterPreferences.SHORT;

  private void setRequestedSpeed(Supplier<AngularVelocity> speed) {
    requestedSpeed = speed;
    setShooterSpeed(requestedSpeed);
  }

  public Command runSetRequestedSpeed(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setRequestedSpeed(speed));
  }

  public Command stopShooter() {
    return Commands.runOnce(() -> shooterStop(), this);
  }

  @Override
  public void periodic() {
    // --- Velocities (cache to avoid redundant CAN calls) ---
    double motor1RPS = shooterMotor1.getVelocity().getValueAsDouble();
    double motor2RPS = shooterMotor2.getVelocity().getValueAsDouble();
    double motor3RPS = shooterMotor3.getVelocity().getValueAsDouble();

    // --- Current draw ---
    double motor1SupplyCurrent = shooterMotor1.getSupplyCurrent().getValueAsDouble();
    double motor2SupplyCurrent = shooterMotor2.getSupplyCurrent().getValueAsDouble();
    double motor3SupplyCurrent = shooterMotor3.getSupplyCurrent().getValueAsDouble();
    double motor1TorqueCurrent = shooterMotor1.getTorqueCurrent().getValueAsDouble();
    double motor2TorqueCurrent = shooterMotor2.getTorqueCurrent().getValueAsDouble();
    double motor3TorqueCurrent = shooterMotor3.getTorqueCurrent().getValueAsDouble();

    // --- Output & supply voltage ---
    double motor1OutputVoltage = shooterMotor1.getMotorVoltage().getValueAsDouble();
    double motor2OutputVoltage = shooterMotor2.getMotorVoltage().getValueAsDouble();
    double motor3OutputVoltage = shooterMotor3.getMotorVoltage().getValueAsDouble();
    double supplyVoltage = shooterMotor1.getSupplyVoltage().getValueAsDouble();

    // --- Temperatures ---
    double motor1Temp = shooterMotor1.getDeviceTemp().getValueAsDouble();
    double motor2Temp = shooterMotor2.getDeviceTemp().getValueAsDouble();
    double motor3Temp = shooterMotor3.getDeviceTemp().getValueAsDouble();

    // --- Closed loop diagnostics ---
    double averageError = filter.calculate(shooterMotor1.getClosedLoopError().getValueAsDouble());
    double error = shooterMotor1.getClosedLoopError().getValueAsDouble();
    double target = shooterMotor1.getClosedLoopReference().getValueAsDouble();
    double range = locked ? ShooterPreferences.WIDE : ShooterPreferences.TIGHT;

    // --- Rolling std dev: sqrt(E[x²] - E[x]²) ---
    double mean = velocityMeanFilter.calculate(motor1RPS);
    double meanSq = velocityMeanSqFilter.calculate(motor1RPS * motor1RPS);
    double stdDev = Math.sqrt(Math.max(0.0, meanSq - mean * mean));

    // --- Ball exit velocity estimation ---
    // wheel RPS = motor RPS * gear ratio; exit vel = wheel RPS * wheel circumference
    double exitVelocityFPS = motor1RPS
        * ShooterPhysicalProperties.MOTOR_TO_WHEEL_GEAR_RATIO
        * Math.PI * (ShooterPhysicalProperties.FLYWHEEL_DIAMETER_INCHES / 12.0);

    // --- Follower sync check ---
    double motor2RPSDelta = motor1RPS - motor2RPS;
    double motor3RPSDelta = motor1RPS - motor3RPS;
    double differentialCurrentDraw = Math.abs(motor1SupplyCurrent - motor2SupplyCurrent);

    // --- High/low error envelope ---
    if (error >= highError) {
      highError = error;
    } else {
      highError -= 1;
    }

    if (error <= lowError) {
      lowError = error;
    } else {
      lowError += 1;
    }

    // --- Settle / lock logic ---
    if (MathUtil.isNear(shooterTarget, target, 1.0) && Math.abs(error) < range) {
      if (settleCount < ShooterPreferences.STABLE_COUNT) {
        settleCount += 1;
      }
    } else {
      settleCount = 0;
    }

    locked = settleCount >= ShooterPreferences.STABLE_COUNT && shooterTarget > 0.0;

    // --- Time to lock ---
    if (locked && !wasLocked) {
      timeToLockSeconds = Timer.getFPGATimestamp() - shooterStartTimestamp;
    }

    // --- Shot counter: locked → unlocked while target is still set ---
    if (!locked && wasLocked && shooterTarget > 0.0) {
      shotCount++;
    }

    wasLocked = locked;

    // --- SmartDashboard ---
    // Velocity
    SmartDashboard.putNumber("Shooter/Motor1RPS", motor1RPS);
    SmartDashboard.putNumber("Shooter/Motor2RPS", motor2RPS);
    SmartDashboard.putNumber("Shooter/Motor3RPS", motor3RPS);
    SmartDashboard.putNumber("Shooter/TargetRPS", shooterTarget);
    SmartDashboard.putNumber("Shooter/Motor1StdDev", stdDev);
    SmartDashboard.putNumber("Shooter/BallExitVelocityFPS", exitVelocityFPS);

    // Follower sync
    SmartDashboard.putNumber("Shooter/Motor2RPSDelta", motor2RPSDelta);
    SmartDashboard.putNumber("Shooter/Motor3RPSDelta", motor3RPSDelta);

    // Supply & output voltage
    SmartDashboard.putNumber("Shooter/SupplyVoltage", supplyVoltage);
    SmartDashboard.putNumber("Shooter/Motor1OutputVoltage", motor1OutputVoltage);
    SmartDashboard.putNumber("Shooter/Motor2OutputVoltage", motor2OutputVoltage);
    SmartDashboard.putNumber("Shooter/Motor3OutputVoltage", motor3OutputVoltage);

    // Current
    SmartDashboard.putNumber("Shooter/Motor1SupplyCurrent", motor1SupplyCurrent);
    SmartDashboard.putNumber("Shooter/Motor2SupplyCurrent", motor2SupplyCurrent);
    SmartDashboard.putNumber("Shooter/Motor3SupplyCurrent", motor3SupplyCurrent);
    SmartDashboard.putNumber("Shooter/DifferentialCurrent", differentialCurrentDraw);
    SmartDashboard.putNumber("Shooter/Motor1TorqueCurrent", motor1TorqueCurrent);
    SmartDashboard.putNumber("Shooter/Motor2TorqueCurrent", motor2TorqueCurrent);
    SmartDashboard.putNumber("Shooter/Motor3TorqueCurrent", motor3TorqueCurrent);

    // Temperature
    SmartDashboard.putNumber("Shooter/Motor1Temp", motor1Temp);
    SmartDashboard.putNumber("Shooter/Motor2Temp", motor2Temp);
    SmartDashboard.putNumber("Shooter/Motor3Temp", motor3Temp);

    // PID diagnostics
    SmartDashboard.putNumber("Shooter/Error", error);
    SmartDashboard.putNumber("Shooter/AverageError", averageError);
    SmartDashboard.putNumber("Shooter/HighError", highError);
    SmartDashboard.putNumber("Shooter/LowError", lowError);
    SmartDashboard.putNumber("Shooter/SettleCount", settleCount);

    // Lock & shot
    SmartDashboard.putBoolean("Shooter/Locked", locked);
    SmartDashboard.putNumber("Shooter/TimeToLockSeconds", timeToLockSeconds);
    SmartDashboard.putNumber("Shooter/ShotCount", shotCount);

    // Speed preset indicators
    SmartDashboard.putBoolean("Shooter/Long Speed", requestedSpeed.get() == Constants.ShooterPreferences.LONG);
    SmartDashboard.putBoolean("Shooter/Medium Speed", requestedSpeed.get() == Constants.ShooterPreferences.MEDIUM);
    SmartDashboard.putBoolean("Shooter/Short Speed", requestedSpeed.get() == Constants.ShooterPreferences.SHORT);
  }

}
