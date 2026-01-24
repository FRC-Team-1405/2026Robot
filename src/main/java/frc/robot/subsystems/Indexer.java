// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Prefs;
import frc.robot.sim.SimProfiles;

public class Indexer extends SubsystemBase {
  // private final TalonFX indexerMotor = new TalonFX(22);

  // private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  // private final NeutralOut m_Brake = new NeutralOut();

  // private void setIndexerSpeed() {
  // indexerMotor.setControl(velocityVoltage.withVelocity(Prefs.INDEXER_VELOCITY));
  // }

  // private void indexerStop() {
  // indexerMotor.setControl(m_Brake);
  // }

  // /** Creates a new Indexer. */
  // public Indexer() {
  // SimProfiles.initIndexer(indexerMotor);
  // }

  // public Command runIndexer() {
  // return this.runEnd(() -> {
  // setIndexerSpeed();
  // }, () -> {
  // indexerStop();
  // });
  // }

  // public Command runStopIndexer() {
  // return runOnce(this::indexerStop);
  // }

  // @Override
  // public void periodic() {
  // }
}
