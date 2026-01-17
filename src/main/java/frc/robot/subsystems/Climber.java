// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX motor = new TalonFX(45);

  @Override
  public void periodic() {
    System.out.println("Motor voltage = " + motor.getMotorVoltage().getValueAsDouble() + " Motor position = "
        + motor.getPosition().getValueAsDouble());
    if (Utils.isSimulation()) {
      motor.setPosition(motor.getPosition().getValueAsDouble() + motor.getMotorVoltage().getValueAsDouble());
      // set motor position to ( motor position as decimal )
      // set motor position to ( current position )
      // set motor position = current position + motor voltage
    }
  }

  public void setupMotors() {
    TalonFXConfiguration Climber_cfg = new TalonFXConfiguration();
    StatusCode Climber_status_slave = StatusCode.StatusCodeNotInitialized;

    SoftwareLimitSwitchConfigs soft = Climber_cfg.SoftwareLimitSwitch;
    soft.ForwardSoftLimitEnable = true;
    soft.ForwardSoftLimitThreshold = 5.1; // mechanism rotations (after gear ratio)
    soft.ReverseSoftLimitEnable = true;
    soft.ReverseSoftLimitThreshold = 0.0;

    for (int i = 0; i < 5; ++i) {
      Climber_status_slave = motor.getConfigurator().apply(Climber_cfg);
      if (Climber_status_slave.isOK())
        break;
    }
    if (!Climber_status_slave.isOK()) {
      System.out.println("Could not configure Climber slaveMotor. Error: " + Climber_status_slave.toString());
    }
  }

  private double MAX_DISTANCE = 3.0;

  public void move(double position) {
    // position = MAX_DISTANCE * (position);
    motor.setControl(new MotionMagicVoltage(position));

  }

  public Climber() {
    setupMotors();
  }

  // function to climb up - motor forward direction
  public void climbUp() {
    motor.set(0.01);
  }

  // function to climb down -- motor backwards direction
  public void climbDown() {
    motor.set(-0.01);
  }

  public void stop() {
    motor.set(0);
  }

}
