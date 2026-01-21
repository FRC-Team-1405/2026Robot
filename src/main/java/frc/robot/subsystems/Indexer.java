// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.sim.SimProfiles;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor = new TalonFX(21);

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private void setIndexerSpeed(Supplier<AngularVelocity> speed) {
    indexerMotor.setControl(velocityVoltage.withVelocity(speed.get()));
  }

  private void indexerStop() {
    indexerMotor.setControl(new VelocityVoltage(0.0));
  }

  /** Creates a new Indexer. */
  public Indexer() {
    SimProfiles.initIndexer(indexerMotor);
  }

  public Command runIndexer(Supplier<AngularVelocity> speed) {
    return this.runEnd(() -> {
      setIndexerSpeed(speed);
    }, () -> {
      indexerStop();
    });
  }

  @Override
  public void periodic() {
  }
}
