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
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.ResetMode;   // Import the new global ResetMode
import com.revrobotics.PersistMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  private final SparkFlexConfig shooterBottomConfig = new SparkFlexConfig();
  private final SparkFlexConfig shooterTopConfig = new SparkFlexConfig();
 
  private final SparkClosedLoopController shooterFrontBottomController;
  private final SparkClosedLoopController shooterBackBottomController;
  private final SparkClosedLoopController shooterFrontTopController;
  private final SparkClosedLoopController shooterBackTopController;

  private final InterpolatingDoubleTreeMap topMotorTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap bottomMotorTable = new InterpolatingDoubleTreeMap();

 
  /** Creates a new Shooter. */
  public Shooter(
   final int FRONT_BOTTOM_SHOOTER_ID, 
   final int FRONT_TOP_SHOOTER_ID,
   final int BACK_BOTTOM_SHOOTER_ID, 
   final int BACK_TOP_SHOOTER_ID,
   final int FRONT_BOTTOM_ENCODER_ID,
   final int FRONT_TOP_ENCODER_ID,
   final int BACK_BOTTOM_ENCODER_ID,
   final int BACK_TOP_ENCODER_ID) {
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

  //setting up the controllers- closed loop
  shooterFrontBottomController = shooterFrontBottomMotor.getClosedLoopController();
  shooterBackBottomController = shooterBackBottomMotor.getClosedLoopController();
  shooterFrontTopController = shooterFrontTopMotor.getClosedLoopController();
  shooterBackTopController = shooterBackTopMotor.getClosedLoopController();
  
  //PID CONFIG
  shooterBottomConfig.closedLoop
  .p(Constants.Shooter.BOTTOM_SHOOTER_P)
  .i(Constants.Shooter.BOTTOM_SHOOTER_I)
  .d(Constants.Shooter.BOTTOM_SHOOTER_D);
 
  shooterTopConfig.closedLoop
  .p(Constants.Shooter.TOP_SHOOTER_P)
  .i(Constants.Shooter.TOP_SHOOTER_I)
  .d(Constants.Shooter.TOP_SHOOTER_D);

  //configuring the motor
   shooterFrontBottomMotor.configure(shooterBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   shooterBackBottomMotor.configure(shooterBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   shooterFrontTopMotor.configure(shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   shooterBackTopMotor.configure(shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
   //NEED TO GATHER DATA
   //add experimental value here (distance, RPM) for top motors
   topMotorTable.put(0.0,0.0);
   //add experimental values here (distance, RPM) for bottom motors
   bottomMotorTable.put(null, null);
    }

//methods
//simple, just stopping motors
  public void stopTopShooterMotors() {
  shooterFrontTopMotor.set(0.0);
  shooterBackTopMotor.set(0.0); 
  }

  public void stopBottomShooterMotors() {
    shooterFrontBottomMotor.set(0.0);
    shooterBackBottomMotor.set(0.0);
  }

  //using the fun tree thing to get the RPM
  public double getTopTargetRPM(double distance) {
    return topMotorTable.get(distance);
  }

  public double getBottomTargetRPM(double distance) {
    return bottomMotorTable.get(distance);
  }
//find out if motors are at correct speeds :) will be important in testing trust
  public boolean frontBottomIsAtSpeed(double distance) {
    double tolerance = 50.0; //idk if this is too much/little who knows who knows 
    return Math.abs(shooterFrontBottomEncoder.getVelocity() - getTopTargetRPM(distance)) < tolerance;
  }

  
  public boolean frontTopIsAtSpeed(double distance) {
    double tolerance = 50.0; //idk if this is too much/little who knows who knows 
    return Math.abs(shooterFrontTopEncoder.getVelocity() - getTopTargetRPM(distance)) < tolerance;
  }
  
  public boolean backBottomIsAtSpeed(double distance) {
    double tolerance = 50.0; //idk if this is too much/little who knows who knows 
    return Math.abs(shooterBackBottomEncoder.getVelocity() - getTopTargetRPM(distance)) < tolerance;
  }
  
  public boolean backTopsAtSpeed(double distance) {
    double tolerance = 50.0; //idk if this is too much/little who knows who knows 
    return Math.abs(shooterBackTopEncoder.getVelocity() - getTopTargetRPM(distance)) < tolerance;
  }


 // setting the shoote rpm based off of the table
  public void setShooterRPM(double distanceMeters) {
  double topRPM = getTopTargetRPM(distanceMeters);
  double bottomRPM = getBottomTargetRPM(distanceMeters);

   shooterFrontTopController.setSetpoint(topRPM, ControlType.kVelocity);
  shooterBackTopController.setSetpoint(topRPM, ControlType.kVelocity);
  shooterFrontBottomController.setSetpoint(bottomRPM, ControlType.kVelocity);
  shooterBackBottomController.setSetpoint(bottomRPM, ControlType.kVelocity);
}


//Starting shooter commands!!

 public Command stopMotors() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          stopTopShooterMotors();
          stopBottomShooterMotors();
        }));
  }
 
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //putting motor velocities on SmartDashboard
    SmartDashboard.putNumber("Front Bottom Shooter Motor Velocity", shooterFrontBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Front Top Shooter Motor Velocity", shooterFrontTopEncoder.getVelocity());
    SmartDashboard.putNumber("Back Bottom Shooter Motor Velocity", shooterBackBottomEncoder.getVelocity());
    SmartDashboard.putNumber("Back Top Shooter Motor Velocity", shooterBackTopEncoder.getVelocity());

  }
}

