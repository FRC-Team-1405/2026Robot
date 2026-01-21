// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.lang.Math;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Prefs;
import frc.robot.sim.SimProfiles;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor = new TalonFX(Constants.CANBus.SHOOTER);

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private final NeutralOut m_Brake = new NeutralOut();

  private int SettleCount = 0;

  private boolean locked = false;

  public boolean isReadyToFire() {
    return locked;
  }

  private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    shooterMotor.setControl(m_VelocityVoltage.withVelocity(speed.get()));
  }

  private void shooterStop() {
    shooterMotor.setControl(m_Brake);
  }

  /** Creates a new Shooter. */
  public Shooter() {
    SimProfiles.initShooter(shooterMotor);
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return this.runEnd(() -> {
      setShooterSpeed(speed);
    }, () -> {
      shooterStop();
    });
  }

  @Override
  public void periodic() {
    double error = shooterMotor.getClosedLoopError().getValueAsDouble();
    double range = locked ? Prefs.WIDE : Prefs.TIGHT;
    if (Math.abs(error) < range) {
      SettleCount += 1;
    } else {
      SettleCount = 0;
    }

    locked = SettleCount >= Prefs.STABLE_COUNT;
  }

}
