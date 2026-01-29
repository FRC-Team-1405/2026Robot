// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AdjustableShooter extends SubsystemBase {
  private final TalonFX adjustableShooterMotor = new TalonFX(Constants.CANBus.ADJUSTABLE_SHOOTER_MOTOR);

  // private final DigitalInput upperLimit = new DigitalInput();
  // private final DigitalInput upperLimit = new DigitalInput();

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_Brake = new NeutralOut();

  public enum ShooterAngle {
    Home(0.0),
    AnglePos(5.0); // this should be a double supplier at some point

    private double anglePos;

    private ShooterAngle(Double anglePos) {
      this.anglePos = anglePos;
    }

    public double getAngle() {
      return this.anglePos;
    }
  };

  public AdjustableShooter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
