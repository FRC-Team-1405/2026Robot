// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.CalibrateTurret;
import frc.robot.commands.FixedShooter;
import frc.robot.commands.ShootWithIndexer;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Define set points for shooting if autos fail
  public static final double rightClimbRPM = Constants.SetPoints.climbRightTargetRPM;
  public static final double leftClimbRPM = Constants.SetPoints.climbLeftTargetRPM;
  public static final double rightTrenchRPM = Constants.SetPoints.trenchRightTargetRPM;
  public static final double leftTrenchRPM = Constants.SetPoints.trenchLeftTargetRPM;
  public static final Rotation2d rightClimbAngle = Constants.SetPoints.climbRightTurretAngle;
  public static final Rotation2d leftClimbAngle = Constants.SetPoints.climbLeftTurretAngle;
  public static final Rotation2d rightTrenchAngle = Constants.SetPoints.trenchRightTurretAngle;
  public static final Rotation2d leftTrenchAngle = Constants.SetPoints.trenchLeftTurretAngle;

  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final PowerDistribution powerDistribution = new PowerDistribution();
  public static final Vision vision = new Vision(
      (visionPose, timestamp, stdDevs) -> {
        swerve.getPoseEstimator().addVisionMeasurement(
            visionPose,
            timestamp,
            VecBuilder.fill(stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)));
      });

  public static final Intake intake = new Intake(Constants.Intake.INTAKE_LIFT_MOTOR_ID,
      Constants.Intake.INTAKE_MOTOR_ID);
  public static final Indexer indexer = new Indexer(Constants.Indexer.INDEXER_MOTOR_ID);
  public static final Turret turret = new Turret("Turret", Constants.Turret.TURRET_MOTOR_ID,
      Constants.Turret.Turret_HALL_EFFECT_ID, null); // TODO obtain transform from robot to turret
  public static final Shooter shooter = new Shooter(
      Constants.Shooter.TOP_LEFT_SHOOTER_ID,
      Constants.Shooter.BOTTOM_LEFT_SHOOTER_ID,
      Constants.Shooter.TOP_RIGHT_SHOOTER_ID,
      Constants.Shooter.BOTTOM_RIGHT_SHOOTER_ID,
      Constants.Shooter.BITTY_SHOOTER_ID);
  public static final FireControl fireControl = new FireControl(() -> swerve.getPose(),
      () -> DriverStation.getAlliance().orElse(Alliance.Blue), () -> new ChassisSpeeds());
  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {
    configureButtonBindings();

    vision.addCamera("name", Constants.Vision.robotToCam1);

    SmartDashboard.putData(swerve.zeroModulesCommand());

    swerve.setDefaultCommand(new SwerveDriveWithGamepad(swerve));

    indexer.setDefaultCommand(new ShootWithIndexer(shooter, indexer));

    SmartDashboard.putData("Reset position", Commands.runOnce(() -> {
      swerve.resetOdometry(Pose2d.kZero);
    }, swerve));
  }

  private void configureButtonBindings() {
    coDriver.START();
    SmartDashboard.putData(new ZeroTurret(turret));
    SmartDashboard.putData(new CalibrateTurret(turret));

    driver.LT().whileTrue(Commands.sequence(intake.putDownIntake(), intake.intakeFuel()));
    driver.LB().whileTrue(intake.extakeFuel());
    driver.RT().whileTrue(shootTestFuelCommand());
    driver.Y().onTrue(intake.putUpIntake());

    // Clear intake/indexer
    coDriver.LT().whileTrue(Commands.startEnd(() -> indexer.indexerBackward(), () -> indexer.stopIndexer(), indexer));
    coDriver.RT().whileTrue(Commands.startEnd(() -> intake.intakeBackward(), () -> intake.stopIntake(), intake));

    // Set positions to shoot from if autos fail
    coDriver.A().onTrue(new FixedShooter(shooter, turret, rightClimbRPM, rightClimbAngle));
    coDriver.B().onTrue(new FixedShooter(shooter, turret, rightTrenchRPM, rightTrenchAngle));
    coDriver.X().onTrue(new FixedShooter(shooter, turret, leftTrenchRPM, leftTrenchAngle));
    coDriver.Y().onTrue(new FixedShooter(shooter, turret, leftClimbRPM, leftClimbAngle));
  }

  public Command shootTestFuelCommand() {
    return Commands.run(
      () -> {
        double targetRPM = fireControl.getShooterRpm();
        shooter.setShooterRPM(targetRPM, targetRPM);
      }, shooter
    );
  }
}