// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MoveMode;

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

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
                MoveMode moveMode = new MoveMode();

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed
                                                                * moveMode.selectSpeedMode().getAsDouble())
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed
                                                                * moveMode.selectSpeedMode().getAsDouble())
                                                .withRotationalRate(moveMode.selectRotationMode(joystick, drivetrain,
                                                                MaxAngularRate).getAsDouble())));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // TODO: maybe uncomment joystick.b()... because IDK if my move algorithm
                // overrides this algorithm
                // joystick.b().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(
                // new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Zeroize/reset the field-centric heading on start and back press.
                joystick.start().and(joystick.back()).onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Select speed mode
                joystick.leftTrigger().onTrue(moveMode.setToSlowMode());
                joystick.rightTrigger().onTrue(moveMode.setToFastMode());
                // for the 2 lines below, ex. Left trigger is held, and right trigger is tapped
                // quickly such that left trigger is still being held after right trigger is
                // tapped. MoveMode.currentSpeedMode would be SLOW, then FAST, *then SLOW
                // again*.
                joystick.leftTrigger().and(joystick.rightTrigger().negate()).onTrue(moveMode.setToSlowMode());
                joystick.rightTrigger().and(joystick.leftTrigger().negate()).onTrue(moveMode.setToFastMode());

                joystick.leftTrigger().or(joystick.rightTrigger()).onFalse(moveMode.setToNormalMode());

                // Select rotation mode
                joystick.y().onTrue(moveMode.setToStandardMode());
                joystick.x().onTrue(moveMode.setToSnakeMode());
                joystick.b().onTrue(moveMode.setToCompassMode());

                joystick.rightBumper().onTrue(new Command() {
                        @Override
                        public void initialize() {
                                joystick.setRumble(RumbleType.kBothRumble, 1);
                        }
                });

                joystick.rightBumper().onFalse(new Command() {
                        @Override
                        public void initialize() {
                                joystick.setRumble(RumbleType.kBothRumble, 0);
                        }
                });

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Then slowly drive forward (away from us) for 5 seconds.
                                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                                .withVelocityY(0)
                                                .withRotationalRate(0))
                                                .withTimeout(5.0),
                                // Finally idle for the rest of auton
                                drivetrain.applyRequest(() -> idle));
        }
}
