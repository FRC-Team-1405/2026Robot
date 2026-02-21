// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;   // Import the new global ResetMode
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {
// view from shooter side
  private final SparkFlex shooterLeftBottomMotor; 
  private final SparkFlex shooterLeftTopMotor;
  private final SparkFlex shooterRightBottomMotor;
  private final SparkFlex shooterRightTopMotor;

private final RelativeEncoder shooterLeftBottomEncoder;
private final RelativeEncoder shooterLeftTopEncoder;
private final RelativeEncoder shooterRightBottomEncoder;
private final RelativeEncoder shooterRightTopEncoder;

 private final SparkClosedLoopController shooterBottomController;
 private final SparkClosedLoopController shooterTopController;
private boolean hardwareFollowConfigured = false;
 private double topTargetRPM = 0.0;
 private double bottomTargetRPM = 0.0;
 

 //RECOVERY TRACKING STUFF 
  
// Top shooter recovery tracking
private boolean topInTolerance = true;      // Is top motor currently within target RPM tolerance?
private long topRecoveryStart = 0;          // Time when top motor fell below tolerance
private final List<Long> topRecoveryTimes = new ArrayList<>();  // List of recorded recovery times

// Bottom shooter recovery tracking
private boolean bottomInTolerance = true;   // Is bottom motor currently within target RPM tolerance?
private long bottomRecoveryStart = 0;       // Time when bottom motor fell below tolerance
private final List<Long> bottomRecoveryTimes = new ArrayList<>(); // List of recorded recovery times
 
  /** Creates a new Shooter. */
  public Shooter(
   final int TOP_LEFT_SHOOTER_ID, 
   final int BOTTOM_LEFT_SHOOTER_ID,
   final int TOP_RIGHT_SHOOTER_ID, 
   final int BOTTOM_RIGHT_SHOOTER_ID
  ) {
   //configuring motors
  shooterLeftBottomMotor = new SparkFlex(BOTTOM_LEFT_SHOOTER_ID, MotorType.kBrushless);
  shooterLeftTopMotor = new SparkFlex(TOP_LEFT_SHOOTER_ID, MotorType.kBrushless);
  shooterRightBottomMotor = new SparkFlex(BOTTOM_RIGHT_SHOOTER_ID, MotorType.kBrushless);
  shooterRightTopMotor = new SparkFlex(TOP_RIGHT_SHOOTER_ID, MotorType.kBrushless);
  
  
  //configurating encoders
  shooterLeftBottomEncoder = shooterLeftBottomMotor.getEncoder();
  shooterLeftTopEncoder = shooterLeftTopMotor.getEncoder();
  shooterRightBottomEncoder = shooterRightBottomMotor.getEncoder();
  shooterRightTopEncoder = shooterRightTopMotor.getEncoder();
    

//CONTROLLER STUFF
  shooterBottomController = shooterLeftBottomMotor.getClosedLoopController();
   SparkFlexConfig shooterBottomConfig = new SparkFlexConfig();
 shooterBottomConfig.closedLoop.feedForward
    .kS(Constants.Shooter.BOTTOM_kS)
    .kV(Constants.Shooter.BOTTOM_kV) 
    .kA(Constants.Shooter.BOTTOM_kA);

  shooterTopController = shooterLeftTopMotor.getClosedLoopController();
 SparkFlexConfig shooterTopConfig = new SparkFlexConfig();
 shooterTopConfig.closedLoop.feedForward
    .kS(Constants.Shooter.TOP_kS)
    .kV(Constants.Shooter.TOP_kV) 
    .kA(Constants.Shooter.TOP_kA);

  //PID CONFIG
  shooterBottomConfig.closedLoop
  .p(Constants.Shooter.BOTTOM_SHOOTER_P)
  .i(Constants.Shooter.BOTTOM_SHOOTER_I)
  .d(Constants.Shooter.BOTTOM_SHOOTER_D);
 
  shooterTopConfig.closedLoop
  .p(Constants.Shooter.TOP_SHOOTER_P)
  .i(Constants.Shooter.TOP_SHOOTER_I)
  .d(Constants.Shooter.TOP_SHOOTER_D);
   
  shooterBottomConfig.closedLoopRampRate(Constants.Shooter.RAMP_RATE);
  shooterTopConfig.closedLoopRampRate(Constants.Shooter.RAMP_RATE);
  

  //configuring the motor
  shooterBottomConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_STALL_LIMIT, Constants.Shooter.SHOOTER_CURRENT_FREE_LIMIT)
    .voltageCompensation(Constants.Shooter.SHOOTER_VOLTAGE_LIMIT);

  shooterTopConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_STALL_LIMIT,Constants.Shooter.SHOOTER_CURRENT_FREE_LIMIT)
    .voltageCompensation(Constants.Shooter.SHOOTER_VOLTAGE_LIMIT);

  shooterLeftTopMotor.configure(shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  shooterLeftBottomMotor.configure(shooterBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  shooterRightBottomMotor.configure(shooterBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  shooterRightTopMotor.configure(shooterTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  SmartDashboard.putNumber("Shooter Test Top RPM", 3000);
  SmartDashboard.putNumber("Shooter Test Bottom RPM", 3000);

    try {
    SparkFlexConfig topFollowerFollow = new SparkFlexConfig();
    SparkFlexConfig bottomFollowerFollow = new SparkFlexConfig();
    // follow the leader and invert the output for the follower
    topFollowerFollow.follow(shooterLeftTopMotor, true);
    bottomFollowerFollow.follow(shooterLeftBottomMotor,true);
    // Don't reset previously-applied safe parameters; only enable follower mode
    shooterRightTopMotor.configure(topFollowerFollow, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    shooterRightBottomMotor.configure(bottomFollowerFollow, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    hardwareFollowConfigured = true;
  } catch (Exception ex) {
    // If the follow configuration isn't available in this REVLib version,
    // we'll fall back to software mirroring (below in periodic()).
    hardwareFollowConfigured = false;
    }
  }

//methods
//simple, just stopping motors
  public void stopTopShooterMotors() {
  shooterLeftTopMotor.set(0.0);
  shooterRightTopMotor.set(0.0); 
  }

  public void stopBottomShooterMotors() {
    shooterRightBottomMotor.set(0.0); 
    shooterLeftBottomMotor.set(0.0); 
  }


// find out if motors are at correct speeds :) will be important in testing trust
//   Check if both top motors are at speed
// Check if both TOP shooter motors are at speed
public boolean topMotorsAtSpeed(double topTargetRPM) {
  return Math.abs(shooterLeftTopEncoder.getVelocity() - topTargetRPM)
          < Constants.Shooter.shooterMotorTolerance
      && Math.abs(shooterRightTopEncoder.getVelocity() - topTargetRPM)
          < Constants.Shooter.shooterMotorTolerance;
}

// Check if both BOTTOM shooter motors are at speed
public boolean bottomMotorsAtSpeed(double bottomRPM) {
 

  return Math.abs(shooterLeftBottomEncoder.getVelocity() - bottomRPM)
          < Constants.Shooter.shooterMotorTolerance;
     
}


 // setting the shoote rpm based off of the table 
  public void setShooterRPM(double topRPM, double bottomRPM) {
  topTargetRPM = topRPM;
  bottomTargetRPM = bottomRPM;

  
  shooterTopController.setSetpoint(topRPM, ControlType.kVelocity);
  
  shooterBottomController.setSetpoint(bottomRPM, ControlType.kVelocity);
}
public void setShooter(double speed) {
shooterLeftTopMotor.set(speed);
shooterLeftBottomMotor.set(speed);
}

public double getTopSetpoint() {
  return shooterTopController.getSetpoint();
}
public double getBottomSetpoint() {
  return shooterBottomController.getSetpoint();
}

//Starting shooter commands!!

 public Command stopMotors() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          stopTopShooterMotors();
          stopBottomShooterMotors();
        }));
  }
  // public Command runMotors() {
  //   return Commands.sequence(
  //     Commands.runOnce(() -> {
      
  //     })
  //   )
  // }

 
 public Command shootCommand(double topRPM, double bottomRPM) {
    return Commands.sequence(
        // Spin up top motors
        Commands.runOnce(() -> {
            shooterTopController.setSetpoint(topRPM, ControlType.kVelocity);
        }, this),

        //  Wait until top motors are at target speed
        Commands.waitUntil(() -> topMotorsAtSpeed(topRPM)),

        //  Spin up bottom motors
        Commands.runOnce(() -> {
            shooterBottomController.setSetpoint(bottomRPM, ControlType.kVelocity);
        }, this)
    );
}

// Command to run the shooter backwards at a set speed
public Command reverseShooter(double RPM) {
    return Commands.runOnce(() -> {
        // Set negative RPM to leaders
        shooterTopController.setSetpoint(-RPM, ControlType.kVelocity);
        shooterBottomController.setSetpoint(-RPM, ControlType.kVelocity);
    }, this);
}

public Command spinUp(double topRPM, double bottomRPM) {
  return Commands.run(
      () -> setShooterRPM(topRPM,bottomRPM),
      this
  );
}
public boolean shooterAtSpeed(double topRPM, double bottomRPM) {
  return  bottomMotorsAtSpeed(bottomRPM)
          && topMotorsAtSpeed(topRPM);
}

public Command manualShooterTest() {
  return Commands.run(
    () -> {
        double topRPM = SmartDashboard.getNumber("Shooter/Target Top RPM", 0);
       
        double bottomRPM = SmartDashboard.getNumber("Shooter/Target Bottom RPM", 0);

        setShooterRPM(topRPM, bottomRPM);
    }, this);
}

 
@Override
public void periodic() {
    double topRPM = shooterLeftTopEncoder.getVelocity();
    double bottomRPM = shooterLeftBottomEncoder.getVelocity();
    double currentTime = System.currentTimeMillis();

    //top recovery 
    if (Math.abs(topRPM - getTopSetpoint()) > Constants.Shooter.shooterMotorTolerance) {
        if (topInTolerance) {  // just dropped below
            topInTolerance = false;
            topRecoveryStart = (long) currentTime;
        }
    } else {  // back within tolerance
        if (!topInTolerance) {
            topInTolerance = true;
            double recoveryTime = currentTime - topRecoveryStart;  // in ms
            topRecoveryTimes.add((long) recoveryTime);
            SmartDashboard.putNumber("Top Shooter Recovery Time (ms)", recoveryTime);
        }
    }

    // bottom recovey
    if (Math.abs(bottomRPM - getBottomSetpoint()) > Constants.Shooter.shooterMotorTolerance) {
        if (bottomInTolerance) { // just dropped below
            bottomInTolerance = false;
            bottomRecoveryStart = (long) currentTime;
        }
    } else { // back within tolerance
        if (!bottomInTolerance) {
            bottomInTolerance = true;
            double recoveryTime = currentTime - bottomRecoveryStart;  // in ms
            bottomRecoveryTimes.add((long) recoveryTime);
            SmartDashboard.putNumber("Bottom Shooter Recovery Time (ms)", recoveryTime);
        }
    }
    SmartDashboard.putNumber("Shooter/Top Volts", shooterLeftTopMotor.getAppliedOutput()*shooterLeftTopMotor.getBusVoltage());

    // Display current velocities
    SmartDashboard.putNumber("Shooter/Top Target RPM", topTargetRPM);
    SmartDashboard.putNumber("Shooter/Bottom Target RPM", bottomTargetRPM);
  
    SmartDashboard.putNumber("Shooter/Top RPM", topRPM);
   SmartDashboard.putNumber( "Shooter/Bottom RPM", bottomRPM);
   SmartDashboard.putNumber("Shooter/Top Applied", shooterRightTopMotor.getAppliedOutput());
  SmartDashboard.putNumber("Shooter/Bottom Applied", shooterRightBottomMotor.getAppliedOutput());
   
}    
}

    // This method will be called once per scheduler run
    //putting motor velocities on SmartDashboard
  
   


  


