// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import java.lang.Math;
import java.security.spec.DSAPrivateKeySpec;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.sim.SimProfiles;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor1 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_2);

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_Brake = new NeutralOut();

  private LinearFilter filter = LinearFilter.movingAverage(50);

  private int settleCount = 0;

  private boolean locked = false;

  public boolean isReadyToFire() {
    return locked;
  }

  public void setShooterMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.0;
    configs.Slot0.kI = 0.07;
    configs.Slot0.kD = 0.02;
    configs.Slot0.kV = 0.12;
    configs.Slot0.kS = 0.0;

    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

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
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(speed.get()));
    settleCount = 0;
    locked = false;
  }

  private void shooterStop() {
    shooterMotor1.setControl(m_Brake);
  }

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor2.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    SimProfiles.initShooter(shooterMotor1);
    SimProfiles.initShooter(shooterMotor2);
    stopShooter();
    setShooterMotor();
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setShooterSpeed(speed), this);
  };

  public Command stopShooter() {
    return Commands.runOnce(() -> shooterStop(), this);
  }

  @Override
  public void periodic() {
    double leaderCurrentDraw = shooterMotor1.getSupplyCurrent().getValueAsDouble();
    double followerCurrentDraw = shooterMotor2.getSupplyCurrent().getValueAsDouble();
    double differentialCurrentDraw = Math.abs(leaderCurrentDraw - followerCurrentDraw);

    double averageError = filter.calculate(shooterMotor1.getClosedLoopError().getValueAsDouble());
    double error = shooterMotor1.getClosedLoopError().getValueAsDouble();
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

    if (Math.abs(error) < range) {
      if (settleCount < ShooterPreferences.STABLE_COUNT) {
        settleCount += 1;
      }
    } else {
      settleCount = 0;
    }

    locked = settleCount >= ShooterPreferences.STABLE_COUNT;

    SmartDashboard.putNumber("Shooter/RPS", shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/LeaderCurrentDraw", leaderCurrentDraw);
    SmartDashboard.putNumber("Shooter/FollowerCurrentDraw", followerCurrentDraw);
    SmartDashboard.putNumber("Shooter/DifferentialCurrentDraw", differentialCurrentDraw);
    SmartDashboard.putNumber("Shooter/Error", error);
    SmartDashboard.putNumber("Shooter/AverageError", averageError);
    SmartDashboard.putNumber("Shooter/HighError", highError);
    SmartDashboard.putNumber("Shooter/LowError", lowError);
    SmartDashboard.putNumber("Shooter/SettleCount", settleCount);
    SmartDashboard.putBoolean("Shooter/Locked", locked);
  }

}
