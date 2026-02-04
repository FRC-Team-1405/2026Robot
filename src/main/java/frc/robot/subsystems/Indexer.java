// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final SparkFlex indexerMotor;
  private final SparkFlexConfig indexerMotorConfig;
  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = new SparkFlex(Constants.Indexer.INDEXER_MOTOR_ID, MotorType.kBrushless);
        indexerMotorConfig = new SparkFlexConfig();
   indexerMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Constants.Indexer.CURRENT_LIMIT)
      .voltageCompensation(Constants.Indexer.VOLTAGE_LIMIT);
    indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
      public void indexerForward(){
    indexerMotor.set(Constants.Indexer.SPEED);
  }

  public void indexerBackward(){
    indexerMotor.set(Constants.Indexer.SPEED * -1);
  }
   public void stopIndexer (){
    indexerMotor.stopMotor();
   }
     public Command shootFuel(){
    return Commands.startEnd(() -> indexerForward(), () -> stopIndexer(), this);
  }

  public Command clearFuel(){
    return Commands.startEnd(() -> indexerBackward(), () -> stopIndexer(), this);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
