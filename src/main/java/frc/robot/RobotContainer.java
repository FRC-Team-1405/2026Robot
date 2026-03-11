// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.CalibrateTurret;
import frc.robot.commands.DefaultTurretCommand;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.commands.ZeroTurret;
import frc.robot.commands.testRPM;
import frc.robot.subsystems.*;

public class RobotContainer {
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
  public static final FireControl fireControl = new FireControl(
                                                      () -> {
                                                              return swerve.getPose().transformBy(new Transform2d(
                                                                Constants.Turret.ROBOT_TO_SHOOTER.getTranslation().toTranslation2d(),
                                                                Constants.Turret.ROBOT_TO_SHOOTER.getRotation().toRotation2d()
                                                              ));
                                                            }, 
                                                      () -> DriverStation.getAlliance().orElse(Alliance.Blue), 
                                                      () -> new ChassisSpeeds());
  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public RobotContainer() {
    configureButtonBindings();


   vision.addCamera("heart", Constants.Vision.robotToHeart);
   vision.addCamera("club", Constants.Vision.robotToClub);
   vision.addCamera("diamond", Constants.Vision.robotToDiamond);
   vision.addCamera("Arducam_OV9281_USB_Camera", Constants.Vision.robotToArudcam);

    SmartDashboard.putData(swerve.zeroModulesCommand());

    swerve.setDefaultCommand(new SwerveDriveWithGamepad(swerve));

    SmartDashboard.putData("Reset position", Commands.runOnce(() -> {
      swerve.resetOdometry(Pose2d.kZero);
    }, swerve));
    turret.setDefaultCommand(Commands.sequence(new CalibrateTurret(turret), new DefaultTurretCommand(turret, fireControl)));
  }

  private void configureButtonBindings() {
    coDriver.START();
    SmartDashboard.putData(new ZeroTurret(turret));
    SmartDashboard.putData(new CalibrateTurret(turret));
    driver.LT().whileTrue(Commands.sequence(intake.putDownIntake(), intake.intakeFuel()));
    driver.LB().whileTrue(intake.extakeFuel());
    driver.RT().whileTrue(shootTestFuelCommand());
    driver.Y().onTrue(intake.putUpIntake());
     coDriver.DUp().whileTrue(intakePushFuel());
     coDriver.DDown().whileTrue(manualIntakeDownCommand());
  }

 
  public Command shootTestFuelCommand() {
    return Commands.sequence(
        shooter.shootCommand(2500, 2500),
        Commands.waitUntil(() -> shooter.shooterAtSpeed(2500, 2500)),
        Commands.run(() -> indexer.indexerForward(), indexer)).finallyDo(() -> {shooter.stopShooterMotors(); indexer.stopIndexer();});
  }
  public Command intakePushFuel() {
  return intake.run(() -> {
    intake.raiseIntakeToJostle();
    intake.intakeToJostle();
  }).finallyDo((()->{intake.stopIntake(); intake.stopLift();}));
}
public Command manualIntakeDownCommand() {
  return intake.run(()-> {
   intake.lowerIntakeManually();
}).finallyDo(()->{ intake.stopLift();});

}
}