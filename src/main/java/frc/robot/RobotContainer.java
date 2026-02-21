// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.commands.Shooter.IndexerShooterStop;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants_OldRobot;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.AprilTags;
import frc.robot.lib.AutoCommands;
import frc.robot.lib.CommandTracker;
import frc.robot.subsystems.AdjustableHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MoveMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionSample;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.Intake;

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
        private final CommandXboxController shooterJoystick = new CommandXboxController(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants_OldRobot.createDrivetrain();

        // Vision publishers
        StructPublisher<Pose2d> cameraEstimatedPosePublisher1 = NetworkTableInstance.getDefault()
                        .getStructTopic("Camera1_EstimatedPose", Pose2d.struct).publish();
        StructPublisher<Pose2d> cameraEstimatedPosePublisher2 = NetworkTableInstance.getDefault()
                        .getStructTopic("Camera2_EstimatedPose", Pose2d.struct).publish();
        List<StructPublisher<Pose2d>> cameraEstimatedPosesPublisher = Arrays.asList(cameraEstimatedPosePublisher1,
                        cameraEstimatedPosePublisher2);

        private Shooter shooter = new Shooter();
        private Indexer indexer = new Indexer();
        public final Climber climber = new Climber();
        public final AdjustableHood hood = new AdjustableHood();
        public final Hopper hopper = new Hopper();
        public final Intake intake = new Intake();
        private final Vision vision = new Vision(Vision.camerasFromConfigs(VisionConstants.CONFIGS));
        MoveMode moveMode = new MoveMode();

        public RobotContainer() {
                configureBindings();

                AutoCommands.registerCommands(drivetrain, climber);
                AprilTags.publishTags(AprilTags.getAprilTagFieldLayout());
        }

        private void configureBindings() {
                // configure operator controls
                Command cmd;

                cmd = intake.runIntakeOut();
                SmartDashboard.putData(cmd);
                operator.povUp().onTrue(cmd);

                cmd = intake.runIntakeIn();
                SmartDashboard.putData(cmd);
                operator.povDown().onTrue(cmd);

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
                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                joystick.leftBumper().onTrue(
                                Commands.either(intake.runPickupStop(), intake.runPickupIn(), intake::isPickupRunning));

                // joystick.rightTrigger()
                // .onTrue(intake.runPickupFuel())
                // .onFalse(intake.runPickupStop("Driver/Intake/Stop"));

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
                // joystick.x().onTrue(moveMode.setToSnakeMode());
                // joystick.b().onTrue(moveMode.setToCompassMode());
                // Reset the field-centric heading on left bumper press.
                joystick.start().and(joystick.back()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);

                shooterJoystick.a().toggleOnTrue(shooter.runShooter(() -> {
                        return ShooterPreferences.INTERMEDIATE;
                }));

                shooterJoystick.x().onTrue(new IndexerShooterStop(shooter, indexer));

                shooterJoystick.b().toggleOnTrue(indexer.runIndexer(() -> {
                        return ShooterPreferences.INDEXER_VELOCITY;
                }));

                shooterJoystick.y().toggleOnTrue(
                                new AutoFire(shooter, indexer, () -> ShooterPreferences.LONG,
                                                () -> ShooterPreferences.INDEXER_VELOCITY));

        }

        public Command getAutonomousCommand() {
                return AutoCommands.getAutonomousCommand();
        }

        public void correctOdometry() {
                // if (SIMULATE_VISION_FAILURES){
                // int percentageFramesToDrop = 80;
                // Random rnd = new Random();

                // if(rnd.nextInt(100) < percentageFramesToDrop){
                // return;
                // }
                // }

                List<VisionSample> visionSamples = vision.flushSamples();
                vision.updateSpeeds(drivetrain.getState().Speeds);
                // System.out.println("vision sample count: " + visionSamples.size());
                for (var sample : visionSamples) {

                        double thetaStddev = 99999.0;
                        if (true /* STRICT_VISION_ORIENTATION_WEIGHTING */) {
                                // if sample weight isn't essentially perfect, don't trust orientation, sample
                                // weighting is perfect when disabled
                                thetaStddev = sample.weight() > 0.9 ? 10.0 : 99999.0;
                        } else {
                                // You will need to TUNE this scalar. A higher value (e.g., 5.0) means less
                                // trust.
                                thetaStddev = 1.0 / sample.weight();
                        }

                        drivetrain.addVisionMeasurement(
                                        sample.pose(),
                                        sample.timestamp(),
                                        VecBuilder.fill(0.1 / sample.weight(), 0.1 / sample.weight(), thetaStddev));
                }

                Pose2d visionPose = null;
                Pose2d odomPose = drivetrain.getState().Pose;
                for (int i = 0; i < 2; i++) {
                        if (i + 1 <= visionSamples.size()) {
                                cameraEstimatedPosesPublisher.get(i).set(visionSamples.get(i).pose());
                                visionPose = visionSamples.get(i).pose();
                        }
                }

                // if (visionPose != null) {
                // double yawError = yawDiffDegrees(visionPose, odomPose);
                // yawErrorPub.set(yawError);
                // }
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