// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.AprilTags;
import frc.robot.lib.AutoCommands;
import frc.robot.lib.CommandTracker;
import frc.robot.subsystems.AdjustableHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Climber climber = new Climber();
        public final AdjustableHood hood = new AdjustableHood();

        public RobotContainer() {
                configureBindings();

                AutoCommands.registerCommands(drivetrain, climber);
                AprilTags.publishTags(AprilTags.getAprilTagFieldLayout());
        }

        private void configureBindings() {
                // configure operator controls
                Command cmd;

                cmd = climber.runExtendClimber();
                SmartDashboard.putData(cmd);
                operator.povUp().onTrue(cmd);

                cmd = climber.runRetractClimber();
                SmartDashboard.putData(cmd);
                operator.povDown().onTrue(cmd);

                cmd = Commands.sequence(climber.runStop(), climber.runStopClaw()).withName("Climber Stop");
                SmartDashboard.putData(cmd);
                operator.x().onTrue(cmd);

                cmd = hood.runSet(Constants.HoodPreferences.SERVO_SHORT).withName("Hood Speed Short");
                SmartDashboard.putData(cmd);
                joystick.a().onTrue(cmd);

                cmd = hood.runSet(Constants.HoodPreferences.SERVO_MEDIUM).withName("Hood Speed Medium");
                SmartDashboard.putData(cmd);
                joystick.b().onTrue(cmd);

                cmd = hood.runSet(Constants.HoodPreferences.SERVO_LONG).withName("Hood Speed Long");
                SmartDashboard.putData(cmd);
                joystick.y().onTrue(cmd);

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return AutoCommands.getAutonomousCommand();
        }

        /** Update NetworkTables with active commands from CommandTracker */
        public static void updateNT() {
                var inst = NetworkTableInstance.getDefault();
                var table = inst.getTable("CommandTracker");

                // Write active commands
                int i = 0;
                for (Command cmd : CommandTracker.getRunning()) {
                        table.getEntry("cmd_" + i).setString(cmd.getName());
                        i++;
                }

                // Clear leftover slots
                for (int j = i; j < 2; j++) {
                        table.getEntry("cmd_" + j).setString("");
                }

                table.getEntry("count").setNumber(CommandTracker.getRunning().size());
        }
}
