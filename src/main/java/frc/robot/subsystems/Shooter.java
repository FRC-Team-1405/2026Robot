// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import java.util.prefs.Preferences;
import java.lang.Math;
import java.lang.annotation.Target;
import java.security.spec.DSAPrivateKeySpec;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.sim.SimProfiles;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor1 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_2);
  private final TalonFX shooterMotor3 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_3);

  private final MotionMagicVelocityVoltage m_VelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final NeutralOut m_Brake = new NeutralOut();

  private LinearFilter filter = LinearFilter.movingAverage(50);

  private int settleCount = 0;

  private double shooterTarget = 0.0;

  private boolean locked = false;

  public boolean isReadyToFire() {
    return locked;
  }

  public void setShooterMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.0; // 0.00
    configs.Slot0.kI = 0.0015; // 0.006
    configs.Slot0.kD = 0.0; // 0.00
    configs.Slot0.kV = 0.12; // 0.149 0.1232
    configs.Slot0.kS = 0.0; // 0.00

    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    configs.MotionMagic.MotionMagicAcceleration = 30; // whatever RobotContainer.
                                                      // shooterJoystick.a()...ShooterPreference is equal to
    // configs.MotionMagic.MotionMagicJerk = 0/*1000*/;

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
    double value = speed.get().in(RotationsPerSecond);
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(speed.get()));
    shooterTarget = speed.get().in(RotationsPerSecond);

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
    stopShooter();
    // setShooterMotor();
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setShooterSpeed(speed), this);
  };

  public Command runShooter() {
    return Commands.runOnce(() -> setShooterSpeed(requestedSpeed), this);
  };

  private Supplier<AngularVelocity> requestedSpeed = () -> Constants.ShooterPreferences.SHORT;

  private void setRequestedSpeed(Supplier<AngularVelocity> speed) {
    if (speed.get() == Constants.ShooterPreferences.LONG) {
      SmartDashboard.putBoolean("Shooter/Long Speed", true);
      SmartDashboard.putBoolean("Shooter/Medium Speed", false);
      SmartDashboard.putBoolean("Shooter/Short Speed", false);
    } else if (speed.get() == Constants.ShooterPreferences.MEDIUM) {
      SmartDashboard.putBoolean("Shooter/Long Speed", false);
      SmartDashboard.putBoolean("Shooter/Medium Speed", true);
      SmartDashboard.putBoolean("Shooter/Short Speed", false);
    } else if (speed.get() == Constants.ShooterPreferences.SHORT) {
      SmartDashboard.putBoolean("Shooter/Long Speed", false);
      SmartDashboard.putBoolean("Shooter/Medium Speed", false);
      SmartDashboard.putBoolean("Shooter/Short Speed", true);
    } else {
      SmartDashboard.putBoolean("Shooter/Long Speed", false);
      SmartDashboard.putBoolean("Shooter/Medium Speed", false);
      SmartDashboard.putBoolean("Shooter/Short Speed", false);
    }
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
    double leaderCurrentDraw = shooterMotor1.getStatorCurrent().getValueAsDouble();
    double followerCurrentDraw2 = shooterMotor2.getStatorCurrent().getValueAsDouble();
    double followerCurrentDraw3 = shooterMotor3.getStatorCurrent().getValueAsDouble();

    double differentialCurrentDraw2 = Math.abs(leaderCurrentDraw - followerCurrentDraw2);
    double differentialCurrentDraw3 = Math.abs(leaderCurrentDraw - followerCurrentDraw3);

    double averageError = filter.calculate(shooterMotor1.getClosedLoopError().getValueAsDouble());
    double error = shooterMotor1.getClosedLoopError().getValueAsDouble();
    double target = shooterMotor1.getClosedLoopReference().getValueAsDouble();
    double highError = 0.0;
    double lowError = 0.0;
    double range = locked ? ShooterPreferences.WIDE : ShooterPreferences.TIGHT; // TODO improve name on "locked"

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

    if (target > 0.0) {
      if (MathUtil.isNear(shooterTarget, target, 1.0) && Math.abs(error) < range) {
        if (settleCount < ShooterPreferences.STABLE_COUNT) {
          settleCount += 1;
        }
      } else {
        settleCount = 0;
      }
    } else {
      settleCount = 0;
    }

    locked = settleCount >= ShooterPreferences.STABLE_COUNT && shooterTarget > 0.0;

    SmartDashboard.putNumber("Shooter/ShooterMotor1RPS", shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/ShooterMotor2RPS", shooterMotor2.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/ShooterMotor3RPS", shooterMotor3.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/LeaderCurrentDraw", leaderCurrentDraw);
    SmartDashboard.putNumber("Shooter/FollowerCurrentDraw2", followerCurrentDraw2);
    SmartDashboard.putNumber("Shooter/FollowerCurrentDraw3", followerCurrentDraw3);
    SmartDashboard.putNumber("Shooter/DifferentialCurrentDraw2", differentialCurrentDraw2);
    SmartDashboard.putNumber("Shooter/DifferentialCurrentDraw3", differentialCurrentDraw3);
    SmartDashboard.putNumber("Shooter/Error", error);
    SmartDashboard.putNumber("Shooter/SettleCount", settleCount);
    SmartDashboard.putNumber("Shooter/AverageError", averageError);
    SmartDashboard.putNumber("Shooter/HighError", highError);
    SmartDashboard.putNumber("Shooter/LowError", lowError);
    SmartDashboard.putNumber("Shooter/SettleCount", settleCount);
    SmartDashboard.putBoolean("Shooter/Locked", locked);
  }

}
