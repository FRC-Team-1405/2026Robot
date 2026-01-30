// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterFrontBottomMotor; 
  private final SparkFlex shooterFrontTopMotor;
  private final SparkFlex shooterBackBottomMotor;
  private final SparkFlex shooterBackTopMotor;
  private final RelativeEncoder shooterFrontBottomEncoder;
  private final RelativeEncoder shooterFrontTopEncoder;
  private final RelativeEncoder shooterBackBottomEncoder;
  private final RelativeEncoder shooterBackTopEncoder;
 
  /** Creates a new Shooter. */
  public Shooter(final int FRONT_BOTTOM_SHOOTER_ID, final int FRONT_TOP_SHOOTER_ID, final int BACK_BOTTOM_SHOOTER_ID, 
  final int BACK_TOP_SHOOTER_ID, final int FRONT_BOTTOM_ENCODER_ID, final int FRONT_TOP_ENCODER_ID, final int BACK_BOTTOM_ENCODER_ID, final int BACK_TOP_ENCODER_ID) {
   //configuring motors
    shooterFrontBottomMotor = new SparkFlex(FRONT_BOTTOM_SHOOTER_ID, MotorType.kBrushless);
   shooterFrontTopMotor = new SparkFlex(FRONT_TOP_SHOOTER_ID, MotorType.kBrushless);
  shooterBackBottomMotor = new SparkFlex(BACK_BOTTOM_SHOOTER_ID, MotorType.kBrushless);
  shooterBackTopMotor = new SparkFlex(BACK_TOP_SHOOTER_ID, MotorType.kBrushless);
  
  //configurating encoders
  shooterFrontBottomEncoder = shooterFrontBottomMotor.getEncoder();
  shooterFrontTopEncoder = shooterFrontTopMotor.getEncoder();
  shooterBackBottomEncoder = shooterBackBottomMotor.getEncoder();
  shooterBackTopEncoder = shooterBackTopMotor.getEncoder();




  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
