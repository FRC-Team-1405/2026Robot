// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.HoodPreferences.HoodAngles;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.DriveToHubDistance;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.constants.FeatureSwitches;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.AprilTags;
import frc.robot.lib.AutoCommands;
import frc.robot.lib.CommandTracker;
import frc.robot.subsystems.AdjustableHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MoveMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveFeatures;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionSample;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.commands.Shooter.AutoFire;

public class RobotContainer {
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(SwerveFeatures.MaxSpeed);

        private final CommandXboxController driverJoystick = new CommandXboxController(0);
        private final CommandXboxController operatorJoystick = new CommandXboxController(1);
        private final CommandXboxController shooterJoystick = new CommandXboxController(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Vision publishers
        StructPublisher<Pose2d> cameraEstimatedPosePublisher1 = NetworkTableInstance.getDefault()
                        .getStructTopic("Camera1_EstimatedPose", Pose2d.struct).publish();
        StructPublisher<Pose2d> cameraEstimatedPosePublisher2 = NetworkTableInstance.getDefault()
                        .getStructTopic("Camera2_EstimatedPose", Pose2d.struct).publish();
        List<StructPublisher<Pose2d>> cameraEstimatedPosesPublisher = Arrays.asList(cameraEstimatedPosePublisher1,
                        cameraEstimatedPosePublisher2);

        public Shooter shooter = new Shooter();
        public Indexer indexer = new Indexer();
        public final Climber climber = new Climber();
        public final AdjustableHood hood = new AdjustableHood();
        public final Hopper hopper = new Hopper();
        public final Intake intake = new Intake();
        private final Vision vision = new Vision(Vision.camerasFromConfigs(VisionConstants.CONFIGS));
        MoveMode moveMode = new MoveMode();

        public RobotContainer() {
                configureBindings();
                // configureBindings_CTReDefault();

                AutoCommands.registerCommands(drivetrain, climber, intake, hopper, indexer, shooter);
                AprilTags.publishTags(AprilTags.getAprilTagFieldLayout());
                drivetrain.initOverridePose();
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

        public static final double JOYSTICK_DEADBAND = 0.10;

        private void configureBindings() {
                // configure operator controls
                Command cmd;

                //
                // Operator Controls
                //
                RumbleJoystick.setPeriodChangeWarningOccasion(operatorJoystick);
                cmd = intake.runIntakeOut();
                // SmartDashboard.putData("Commands/RunIntakeOut", cmd);
                operatorJoystick.povUp().onTrue(cmd);

                cmd = intake.runIntakeIn();
                // SmartDashboard.putData("Commands/RunIntakeIn", cmd);
                operatorJoystick.povDown().onTrue(cmd);

                operatorJoystick.povRight().onTrue(new InstantCommand(() -> shooter.increaseDistanceForSpeed()));
                operatorJoystick.povLeft().onTrue(new InstantCommand(() -> shooter.descreaseDistanceForSpeed()));

                operatorJoystick.y().onTrue(new SetHoodPosition(hood, HoodAngles.SHORT));
                operatorJoystick.b().onTrue(new SetHoodPosition(hood, HoodAngles.MEDIUM));
                operatorJoystick.a().onTrue(new SetHoodPosition(hood, HoodAngles.LONG));

                operatorJoystick.y().onTrue(shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT));
                operatorJoystick.b().onTrue(shooter.runSetRequestedSpeed(() -> ShooterPreferences.MEDIUM));
                operatorJoystick.a().onTrue(shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG));
                operatorJoystick.rightBumper().onTrue(
                                Commands.sequence(shooter.stopShooter(), indexer.runStopIndexer()));
                operatorJoystick.leftBumper().toggleOnTrue(intake.runIntakeCenter());

                // Bump mode (for crossing the bump)
                operatorJoystick.leftTrigger().onTrue(moveMode.setToBumpMode(drivetrain));

                // Exit Bump Mode
                operatorJoystick.rightTrigger().onTrue(moveMode.setToNormalMode());

                //
                // Driver Controls
                //
                RumbleJoystick.setPeriodChangeOccasion(driverJoystick);
                drivetrain.registerTelemetry(logger::telemeterize);

                // cmd = SwerveFeatures.teleopDriveCommand(drivetrain, moveMode,
                // driverJoystick).withName("Teleop Drive");
                // SmartDashboard.putData("Commands/TeleopDrive", cmd);
                // drivetrain.setDefaultCommand(
                // // Drivetrain will execute this command periodically
                // cmd);

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> SwerveFeatures.drive
                                                .withVelocityX(SwerveFeatures.MaxSpeed
                                                                * moveMode.selectSpeedMode(driverJoystick::getLeftY,
                                                                                true)
                                                                                .getAsDouble())
                                                .withVelocityY(SwerveFeatures.MaxSpeed
                                                                * moveMode.selectSpeedMode(
                                                                                driverJoystick::getLeftX, false)
                                                                                .getAsDouble())
                                                .withRotationalRate(
                                                                moveMode.selectRotationMode(driverJoystick, drivetrain,
                                                                                SwerveFeatures.MaxAngularRate)
                                                                                .getAsDouble())));

                // Zeroize/reset the field-centric heading on start and back press.
                driverJoystick.start().and(driverJoystick.back()).onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Select speed mode
                driverJoystick.leftTrigger().onTrue(moveMode.setToSlowMode());
                driverJoystick.rightTrigger().onTrue(moveMode.setToFastMode());

                // for the 2 lines below, ex. Left trigger is held, and right trigger is tapped
                // quickly such that left trigger is still being held after right trigger is
                // tapped. MoveMode.currentSpeedMode would be SLOW, then FAST, *then SLOW
                // again*.
                driverJoystick.leftTrigger().and(
                                driverJoystick.rightTrigger().negate()).onTrue(moveMode.setToSlowMode());
                driverJoystick.rightTrigger().and(
                                driverJoystick.leftTrigger().negate()).onTrue(moveMode.setToFastMode());

                driverJoystick.leftTrigger().or(driverJoystick.rightTrigger()).onFalse(moveMode.setToNormalMode());

                // Select rotation mode
                // driverJoystick.y().onTrue(moveMode.setToStandardMode());
                // joystick.x().onTrue(moveMode.setToSnakeMode());
                // joystick.b().onTrue(moveMode.setToCompassMode());

                // Brake mode
                // cmd = drivetrain.applyRequest(() -> brake).withName("Brake Mode");
                // SmartDashboard.putData("Commands/BrakeMode", cmd);
                driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));

                // Auto Align
                // driverJoystick.x()
                // .whileTrue(
                // Commands.either(Commands.defer(() -> drivetrain.driveToPose(
                // () -> Optional.of(
                // FieldConstants.BLUE_HUB_SHOOT_CLOSE),
                // true), Set.of(drivetrain)),
                // Commands.none(),
                // MoveMode.inAllianceZone(drivetrain)));

                // Command driveToHubCommand = new DriveToHubDistance(drivetrain,
                // FieldConstants.ALLIANCE_HUB_POSITION,
                // ShooterPreferences.MEDIUM_DISTANCE);
                Supplier<Command> driveToDistanceCommand = () -> new DriveToHubDistance(drivetrain,
                                FieldConstants.ALLIANCE_HUB_POSITION,
                                shooter.getDistanceFromSpeed());

                driverJoystick.x()
                                .whileTrue(
                                                Commands.either(Commands.defer(driveToDistanceCommand,
                                                                Set.of(drivetrain)),
                                                                Commands.none(),
                                                                MoveMode.inAllianceZone(drivetrain)));

                // Point at hub toggle (STANDARD ↔ POINT or POINT_VELOCITY_COMPENSATED).
                // Which point mode is active is controlled by the
                // "MoveMode/Use Velocity Compensated Point" NT toggle in Elastic Dashboard.
                driverJoystick.y().onTrue(moveMode.togglePointMode());

                Trigger hopperTrigger = new Trigger(() -> {
                        return indexer.isIndexerRunning();
                });
                hopperTrigger.onTrue(hopper.runForwardHopper());
                hopperTrigger.onFalse(hopper.runStopHopper());

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Run Intake (Pickup)
                // driverJoystick.leftBumper().onFalse(intake.runPickupStop());
                driverJoystick.leftBumper().whileTrue(intake.runPickupIn());

                // Shoot
                if (FeatureSwitches.BRAKE_WHILE_SHOOTING) {
                        // final Command shootAndBrakeCommand = Commands.parallel(
                        // AutoFire.teleop(shooter, indexer, hopper,
                        // () -> ShooterPreferences.INDEXER_VELOCITY),
                        // SwerveFeatures.brakeWhenStationaryOrDrive(drivetrain, moveMode,
                        // driverJoystick));
                        // // SmartDashboard.putData("Commands/AutoFireWithBrakeAssist",
                        // // shootAndBrakeCommand);
                        // driverJoystick.rightBumper().whileTrue(shootAndBrakeCommand);
                        // driverJoystick.rightBumper().onFalse(
                        // Commands.parallel(new InstantCommand(() -> shootAndBrakeCommand.end(true)),
                        // indexer.runStopIndexer()));
                } else {
                        final Command shootCommand = new AutoFire.TeleopFireCommand(shooter, indexer, hopper,
                                        () -> ShooterPreferences.INDEXER_VELOCITY);
                        final Command brakeCommand = drivetrain.applyRequest(() -> brake);
                        // SmartDashboard.putData("Commands/AutoFire", shootCommand);
                        driverJoystick.rightBumper()
                                        .whileTrue(Commands.parallel(shootCommand,
                                                        brakeCommand));
                        // driverJoystick.rightBumper().onFalse(
                        // indexer.runStopIndexer());
                }

                //
                // Shooter Joystick (DEBUG) Controls
                //
                // A: spin up to whatever value is set in the Shooter/TestTargetRPS dashboard
                // slider
                // shooterJoystick.a().onTrue(shooter.runShooterAtTestRPS());
                // // B: stop shooter
                // shooterJoystick.b().onTrue(shooter.stopShooter());
                // // Preset speeds
                // shooterJoystick.y().onTrue(shooter.runSetRequestedSpeed(() ->
                // ShooterPreferences.SHORT));
                // shooterJoystick.x().onTrue(shooter.runSetRequestedSpeed(() ->
                // ShooterPreferences.MEDIUM));
                // // shooterJoystick.a().onTrue(shooter.runSetRequestedSpeed(() ->
                // // ShooterPreferences.LONG));
                // // Fire + stop
                // shooterJoystick.rightBumper().onTrue(
                // AutoFire.teleop(shooter, indexer, hopper,
                // () -> ShooterPreferences.INDEXER_VELOCITY));
                // shooterJoystick.rightBumper().onFalse(indexer.runStopIndexer());
        }

        private void configureBindings_CTReDefault() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> SwerveFeatures.drive
                                                .withVelocityX(-driverJoystick.getLeftY() * SwerveFeatures.MaxSpeed) // Drive
                                                // forward
                                                // with
                                                // negative
                                                // Y
                                                // (forward)
                                                .withVelocityY(-driverJoystick.getLeftX() * SwerveFeatures.MaxSpeed) // Drive
                                                                                                                     // left
                                                                                                                     // with
                                                // negative X
                                                // (left)
                                                .withRotationalRate(-driverJoystick.getRightX()
                                                                * SwerveFeatures.MaxAngularRate) // Drive
                                // counterclockwise
                                // with
                                // negative
                                // X
                                // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
                                new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverJoystick.start().and(driverJoystick.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverJoystick.start().and(driverJoystick.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public static double applyDeadband(double value) {
                return (Math.abs(value) < JOYSTICK_DEADBAND) ? 0.0 : value;
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

        public static void publishRobotData() {
                SmartDashboard.putNumber("Battery/BatteryVoltage", RobotController.getBatteryVoltage());
                SmartDashboard.putNumber("Battery/BrownoutVoltage", RobotController.getBrownoutVoltage());
        }
}
