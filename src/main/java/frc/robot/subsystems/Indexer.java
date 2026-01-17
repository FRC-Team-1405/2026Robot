// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor = new TalonFX(21);

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private void setIndexerSpeed(DoubleSupplier speed) {
    indexerMotor.setControl(velocityVoltage.withVelocity(speed.getAsDouble()));
    // indexerMotor.set(speed.getAsDouble());
  }

  private void indexerStop() {
    indexerMotor.setControl(new VelocityVoltage(0.0));
  }

  /** Creates a new Indexer. */
  public Indexer() {
  }

  public Command runIndexer(DoubleSupplier speed) {
    return this.runEnd(() -> {
      setIndexerSpeed(speed);
    }, () -> {
      indexerStop();
    });
  }

  @Override
  public void periodic() {
    System.out.println("indexer motor voltage =" + indexerMotor.getMotorVoltage().getValueAsDouble());
    System.out.println("indexer motor voltage =" + indexerMotor.getMotorVoltage().getValueAsDouble());
  }
}
