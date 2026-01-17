// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor = new TalonFX(Constants.CANBus.SHOOTER);

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);

  private final NeutralOut m_Brake = new NeutralOut();

  private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    shooterMotor.setControl(m_VelocityVoltage.withVelocity(speed.get()));
  }

  private void shooterStop() {
    shooterMotor.setControl(m_Brake);
  }

  /** Creates a new Shooter. */
  public Shooter() {
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
    // This method will be called once per scheduler run
  }
}
