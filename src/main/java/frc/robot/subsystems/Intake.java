// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  // assuming these are singletons
  private final SparkMax liftMotor;
  private final SparkMax intakeMotor;
  private final SparkLimitSwitch liftLimitSwitchUp;
  private final SparkLimitSwitch liftLimitSwitchDown;

  private final double intakeLiftSpeed;
  private double intakeSpeed;


  public Intake(double speed) {
    liftMotor = new SparkMax(Constants.Intake.INTAKE_LIFT_MOTOR_ID, null); // Not sure what type is yet
    intakeMotor = new SparkMax(Constants.Intake.INTAKE_MOTOR_ID, null); //  ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
    liftLimitSwitchUp = liftMotor.getForwardLimitSwitch();
    liftLimitSwitchDown = liftMotor.getReverseLimitSwitch();
    intakeLiftSpeed = Constants.Intake.INTAKE_LIFT_SPEED;
    intakeSpeed = speed;
  }

  public void lowerIntake(){
    liftMotor.set(intakeLiftSpeed * -1);
  }

  public void raiseIntake(){
    liftMotor.set(intakeLiftSpeed);
  }

  public void intakeForward(){
    intakeMotor.set(intakeSpeed);
  }

  public void intakeBackward(){
    intakeMotor.set(intakeSpeed * -1);
  }

  public boolean isIntakeUp(){
    return liftLimitSwitchUp.isPressed();
  }

  public boolean isIntakeDown(){
    return liftLimitSwitchDown.isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
