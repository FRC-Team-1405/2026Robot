package frc.robot.commands.AutoPilot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.therekrab.autopilot.APConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.DriveToHubDistance;
import frc.robot.commands.PidToPose.PidToPoseCommand;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.AprilTags;
import frc.robot.lib.AutoCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CommandsForAutoPilot {
        // TODO: integrate shooting positions dropdown into all autos, include the shoot
        // from distance in the dropdown
        // TODO: add a start position dropdown, integrate into all autos

        // Velocity is max speed overall/ in a sec, how much can position change
        // Acceleration in a second, how much can velocity change
        // Jerk is how fast it starts and stops

        private static final APConstraints bumpConstraints = new APConstraints()
                        .withAcceleration(2.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(2.0);
        // .withJerk(5.0);

        private static final APConstraints testBumpConstraints = new APConstraints()
                        .withAcceleration(2.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(2.0);

        // TODO: Adjust
        private static final APConstraints fullFieldConstraints = new APConstraints()
                        .withAcceleration(2.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(2.0);

        private static final TrapezoidProfile.Constraints centerHarvestConstraint = new TrapezoidProfile.Constraints(
                        0.5, 0.5);
        // .withJerk(67.0);

        // Rotations
        // Positive rotations are CCW:
        // https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
        public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
        public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);

        // Poses
        // off blue center only used for Right Start Depot Score
        public static Supplier<Pose2d> offBlueCenter1 = () -> new Pose2d(2, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> offBlueCenter2 = () -> new Pose2d(0.40, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> feedingStation = () -> FieldConstants.BLUE_FEED_ROBOT_POSITION;
        public static Supplier<Pose2d> rightLoadInZone = () -> new Pose2d(5.75, 2.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftLoadInZone = () -> new Pose2d(5.75, 5.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> quadLeft = () -> new Pose2d(7.5, 3.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> quadRight = () -> new Pose2d(7.5, 6.5, Rotation2d.fromDegrees(270));

        // Start Poses
        public static Supplier<Pose2d> startRightFaceIn = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> startLeftFaceIn = () -> new Pose2d(3.55, 7.65, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> startRightFaceFront = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(0));

        // Bump Poses
        // Left Bump
        private static double BUMP_RIGHT_FIELD_SIDE = 6.0;

        private static double BUMP_LEFT_ALLIANCE_SIDE = 3.3;
        public static Supplier<Pose2d> leftBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(270));

        public static Supplier<Pose2d> leftBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(180));
        // Right Bump
        public static Supplier<Pose2d> rightBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(90));
        // TODO:Remove Dos
        public static Supplier<Pose2d> rightBump_AllianceToFieldStartDos = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> rightBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));

        public static Supplier<Pose2d> rightBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> rightBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        // TODO: fix this
        // Shooter Poses
        public static Supplier<Pose2d> FrontHubShoot = () -> new Pose2d(3.069361, 4.034638, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeIN_FrontHubShoot = () -> new Pose2d(2.069361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeOUT_FrontHubShoot = () -> new Pose2d(3.069361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> RightHubShoot = () -> new Pose2d(3.069361, 3.034638, Rotation2d.fromDegrees(0));
        // Center Poses
        // y position to start center harvesting on the right side
        private static double RIGHT_START_HARVEST_HORIZONTAL_POINT = 1;
        // was 3 and 5.5
        private static double RIGHT_END_HARVEST_HORIZONTAL_POINT = 6;
        private static double LEFT_START_HARVEST_HORIZONTAL_POINT = 7;
        // was 3 and 5.5
        private static double LEFT_END_HARVEST_HORIZONTAL_POINT = 2;

        public static Supplier<Pose2d> farRightCenter = () -> new Pose2d(8.25,
                        RIGHT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> farLeftCenter = () -> new Pose2d(8.25,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(0));

        // centerRightIntakeStart was (7.75, 1, 90)
        public static Supplier<Pose2d> centerRightIntakeStart = () -> new Pose2d(7.45,
                        RIGHT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(90));
        // centerRightIntakeEnd was (7.75, 6, 90)
        public static Supplier<Pose2d> centerRightIntakeEnd = () -> new Pose2d(7.45,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(90));
        // rotation was 180
        public static Supplier<Pose2d> centerRightIntakeEndLookHub = () -> new Pose2d(7.75,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(180));
        // centerRightIntakeStart was (7.75, 7, 270)
        public static Supplier<Pose2d> centerLeftIntakeStart = () -> new Pose2d(7.75,
                        LEFT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(270));
        // centerLeftIntakeEnd was (7.75, 2, 270)
        public static Supplier<Pose2d> centerLeftIntakeEnd = () -> new Pose2d(7.75,
                        LEFT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(270));

        // TODO:Fix lookHub
        public static Supplier<Pose2d> centerLeftIntakeEndLookHub = () -> new Pose2d(7.75,
                        LEFT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(180));

        // Depot Poses
        // wall pose is 0.40
        public static Supplier<Pose2d> depotFaceIn = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftOfDepotFaceIn = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> midOfDepotFaceIn = () -> new Pose2d(0.50, 6, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> rightOfDepotFaceIn = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> depotFaceOut = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> leftOfDepotFaceOut = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> midOfDepotFaceOut = () -> new Pose2d(0.50, 6, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> rightOfDepotFaceOut = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(90));

        // Constant Poses
        // Center Pose is 8,4, Blue Center Pose is 2,4,90
        public static Supplier<Pose2d> blueCenter = () -> new Pose2d(2, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> centerOfField = () -> new Pose2d(8, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> fourMeters = () -> new Pose2d(4, 0, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> preClimbPosition = () -> new Pose2d(2.5, 4, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> climbPosition = () -> new Pose2d(1.5, 4, Rotation2d.fromDegrees(180));

        public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber,
                        Intake intake,
                        Hopper hopper,
                        Indexer indexer,
                        Shooter shooter) {
                /* Commands */
                // Uses command suppliers instead of commands so that we can reuse the same
                // command in an autonomous

                Supplier<Command> MoveTo_blueCenter = () -> new PidToPoseCommand.Builder(
                                () -> blueCenter.get(), drivetrain, "MoveTo_blueCenter")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_CenterTransit = () -> new PidToPoseCommand.Builder(
                                () -> FieldConstants.BLUE_HUB_SHOOT_CLOSE, drivetrain, "MoveTo_blueCenter")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerOfField = () -> new PidToPoseCommand.Builder(
                                () -> centerOfField.get(), drivetrain, "MoveTo_centerOfField")
                                .withFlipPoseForAlliance(true)
                                .build();

                // Feeding station Movements
                Supplier<Command> MoveTo_feedingStation = () -> new PidToPoseCommand.Builder(
                                () -> feedingStation.get(), drivetrain, "MoveTo_feedingStation")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_feedingStation_leftStart = () -> Commands.sequence(MoveTo_CenterTransit.get(),
                                MoveTo_feedingStation.get());
                Supplier<Command> MoveTo_feedingStation_centerStart = () -> Commands.sequence(
                                MoveTo_CenterTransit.get(),
                                MoveTo_feedingStation.get());

                Supplier<Command> MoveTo_fourMeters = () -> new PidToPoseCommand.Builder(
                                () -> fourMeters.get(), drivetrain, "MoveTo_fourMeters")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_preClimbPosition = () -> new PidToPoseCommand.Builder(
                                () -> preClimbPosition.get(), drivetrain, "MoveTo_preClimbPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_climbPosition = () -> new PidToPoseCommand.Builder(
                                () -> climbPosition.get(), drivetrain, "MoveTo_climbPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Shooter
                Supplier<Command> MoveTo_FrontHubShoot = () -> new DriveToHubDistance(drivetrain,
                                FieldConstants.ALLIANCE_HUB_POSITION,
                                shooter.getDistanceFromSpeed());

                Supplier<Command> MoveTo_requestedSpeedDistanceToHub = () -> new DriveToHubDistance(drivetrain,
                                FieldConstants.ALLIANCE_HUB_POSITION,
                                shooter.getDistanceFromSpeed());
                Supplier<Command> MoveTo_ClosestShootingPosition_MEDIUM = () -> Commands.sequence(
                                new InstantCommand(() -> shooter
                                                .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.MEDIUM)),
                                MoveTo_requestedSpeedDistanceToHub.get());
                Supplier<Command> MoveTo_ClosestShootingPosition_LONG = () -> Commands.sequence(
                                new InstantCommand(() -> shooter
                                                .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.LONG)),
                                MoveTo_requestedSpeedDistanceToHub.get());

                Supplier<Command> MoveTo_IntakeIN_FrontHubShoot = () -> new PidToPoseCommand.Builder(
                                () -> IntakeIN_FrontHubShoot.get(), drivetrain, "MoveTo_FrontHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_IntakeOUT_FrontHubShoot = () -> new PidToPoseCommand.Builder(
                                () -> IntakeOUT_FrontHubShoot.get(), drivetrain, "MoveTo_FrontHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_RightHubShoot = () -> new PidToPoseCommand.Builder(
                                () -> RightHubShoot.get(), drivetrain, "MoveTo_RightHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Center Harvest(s)
                Supplier<Command> MoveTo_centerRightIntakeStart = () -> new PidToPoseCommand.Builder(
                                () -> centerRightIntakeStart.get(), drivetrain, "MoveTo_centerRightIntakeStart")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerRightIntakeEnd = () -> new PidToPoseCommand.Builder(
                                () -> centerRightIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(centerHarvestConstraint)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeStart = () -> new PidToPoseCommand.Builder(
                                () -> centerLeftIntakeStart.get(), drivetrain, "MoveTo_centerLeftIntakeStart")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeEnd = () -> new PidToPoseCommand.Builder(
                                () -> centerLeftIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(centerHarvestConstraint)
                                .build();

                Supplier<Command> MoveAndDeploy_centerRightIntakeStart = () -> Commands.parallel(
                                MoveTo_centerRightIntakeStart.get(),
                                intake.runIntakeOut());
                Supplier<Command> MoveAndDeploy_centerLeftIntakeStart = () -> Commands.parallel(
                                MoveTo_centerLeftIntakeStart.get(),
                                intake.runIntakeOut());
                // Supplier<Command> MoveToPickup_centerRightIntakeEnd = () ->
                // MoveTo_centerRightIntakeEnd.get()
                // .alongWith(intake.runPickupIn());

                Supplier<Command> MoveToPickup_centerRightIntakeEnd = () -> Commands
                                .deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn());

                Supplier<Command> MoveToPickup_centerLeftIntakeEnd = () -> Commands
                                .deadline(MoveTo_centerLeftIntakeEnd.get(), intake.runPickupIn());
                // deadline runs in
                // parrallel until the
                // first command
                // finishes
                Supplier<Command> MoveTo_quadLeft = () -> new PidToPoseCommand.Builder(
                                () -> quadLeft.get(), drivetrain, "MoveTo_quadLeft")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_quadRight = () -> new PidToPoseCommand.Builder(
                                () -> quadRight.get(), drivetrain, "MoveTo_quadRight")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveToPickup_quadLeft = () -> Commands
                                .deadline(MoveTo_quadLeft.get(), intake.runPickupIn());

                Supplier<Command> MoveToPickup_quadRight = () -> Commands
                                .deadline(MoveTo_quadRight.get(), intake.runPickupIn());

                Supplier<Command> MoveTo_centerRightIntakeEndLookHub = () -> new PidToPoseCommand.Builder(
                                () -> centerRightIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                                .withFlipPoseForAlliance(true)
                                .withWaitSeconds(1)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeEndLookHub = () -> new PidToPoseCommand.Builder(
                                () -> centerLeftIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                                .withFlipPoseForAlliance(true)
                                .withWaitSeconds(1)
                                .build();

                Supplier<Command> MoveToIntakeUp_centerRightIntakeEndLookHub = () -> Commands
                                .parallel(MoveTo_centerRightIntakeEndLookHub.get(), intake.runIntakeCenter());

                Supplier<Command> MoveToIntakeUp_centerLeftIntakeEndLookHub = () -> Commands
                                .parallel(MoveTo_centerLeftIntakeEndLookHub.get(), intake.runIntakeCenter());

                Supplier<Command> MoveToIntakeUp_quadLeft = () -> Commands
                                .parallel(MoveTo_quadLeft.get(), intake.runIntakeCenter());

                Supplier<Command> MoveToIntakeUp_quadRight = () -> Commands
                                .parallel(MoveTo_quadRight.get(), intake.runIntakeCenter());

                Supplier<Command> MoveTo_rightLoadInZone = () -> new PidToPoseCommand.Builder(
                                () -> rightLoadInZone.get(), drivetrain, "MoveTo_rightLoadInZone")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftLoadInZone = () -> new PidToPoseCommand.Builder(
                                () -> leftLoadInZone.get(), drivetrain, "MoveTo_leftLoadInZone")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Depot
                Supplier<Command> MoveTo_depotFaceIn = () -> new PidToPoseCommand.Builder(
                                () -> depotFaceIn.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceIn = () -> new PidToPoseCommand.Builder(
                                () -> leftOfDepotFaceIn.get(), drivetrain, "MoveTo_leftOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceIn = () -> new PidToPoseCommand.Builder(
                                () -> midOfDepotFaceIn.get(), drivetrain, "MoveTo_midOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceIn = () -> new PidToPoseCommand.Builder(
                                () -> rightOfDepotFaceIn.get(), drivetrain, "MoveTo_rightOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depotFaceOut = () -> new PidToPoseCommand.Builder(
                                () -> depotFaceOut.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceOut = () -> new PidToPoseCommand.Builder(
                                () -> leftOfDepotFaceOut.get(), drivetrain, "MoveTo_leftOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceOut = () -> new PidToPoseCommand.Builder(
                                () -> midOfDepotFaceOut.get(), drivetrain, "MoveTo_midOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceOut = () -> new PidToPoseCommand.Builder(
                                () -> rightOfDepotFaceOut.get(), drivetrain, "MoveTo_rightOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_offBlueCenter1 = () -> new PidToPoseCommand.Builder(
                                () -> offBlueCenter1.get(), drivetrain, "MoveTo_offBlueCenter1")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_offBlueCenter2 = () -> new PidToPoseCommand.Builder(
                                () -> offBlueCenter2.get(), drivetrain, "MoveTo_offBlueCenter2")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_selectedShootPosition = () -> new PidToPoseCommand.Builder(
                                AutoCommands.getShootPosition(), drivetrain, "MoveTo_selectedShootPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                // right from the driver station view
                // Bump Stuff

                Supplier<Command> MoveTo_leftBump_AllianceToFieldStart = () -> new PidToPoseCommand.Builder(
                                () -> leftBump_AllianceToFieldStart.get(), drivetrain,
                                "MoveTo_leftBump_AllianceToFieldStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_leftBump_AllianceToFieldEnd = () -> new PidToPoseCommand.Builder(
                                () -> leftBump_AllianceToFieldEnd.get(), drivetrain,
                                "MoveTo_leftBump_AllianceToFieldEnd")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_rightBump_AllianceToFieldStart = () -> new PidToPoseCommand.Builder(
                                () -> rightBump_AllianceToFieldStart.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_rightBump_AllianceToFieldStartDos = () -> new PidToPoseCommand.Builder(
                                () -> rightBump_AllianceToFieldStartDos.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldStartDos")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                // Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new
                // PidToPoseCommand.Builder(
                // () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
                // "MoveTo_rightBump_AllianceToFieldEnd")
                // .withFlipPoseForAlliance(true)
                // // .withConstraints(testBumpConstraints)
                // .build();

                Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new PidToPoseCommand.Builder(
                                () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldEnd")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(testBumpConstraints)
                                .build();

                Supplier<Command> MoveTo_leftBump_FieldToAllianceStart = () -> new PidToPoseCommand.Builder(
                                () -> leftBump_FieldToAllianceStart.get(), drivetrain,
                                "MoveTo_leftBump_FieldToAllianceStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_leftBump_FieldToAllianceEnd = () -> new PidToPoseCommand.Builder(
                                () -> leftBump_FieldToAllianceEnd.get(), drivetrain,
                                "MoveTo_leftBump_FieldToAllianceEnd")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_rightBump_FieldToAllianceStart = () -> new PidToPoseCommand.Builder(
                                () -> rightBump_FieldToAllianceStart.get(), drivetrain,
                                "MoveTo_rightBump_FieldToAllianceStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(testBumpConstraints)
                                .build();
                Supplier<Command> MoveTo_rightBump_FieldToAllianceEnd = () -> new PidToPoseCommand.Builder(
                                () -> rightBump_FieldToAllianceEnd.get(), drivetrain,
                                "MoveTo_rightBump_FieldToAllianceEnd")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(testBumpConstraints)
                                .build();

                // SHOOTING
                // TODO: Figure out why isn't working
                Supplier<Command> _shoot = () -> Commands
                                .parallel(shooter.runShooterAuto(() -> Constants.ShooterPreferences.MEDIUM),
                                                indexer.runIndexer())
                                .withTimeout(Seconds.of(3))
                                .andThen(Commands.parallel(shooter.stopShooter(), indexer.runStopIndexer()));
                Supplier<Command> MEDIUM_shoot = () -> Commands
                                .parallel(shooter.runShooterAuto(() -> Constants.ShooterPreferences.MEDIUM),
                                                indexer.runIndexer())
                                .withTimeout(Seconds.of(10))
                                .andThen(Commands.parallel(shooter.stopShooter(), indexer.runStopIndexer()));
                Supplier<Command> SHORT_shoot = () -> Commands
                                .parallel(shooter.runShooterAuto(() -> Constants.ShooterPreferences.SHORT),
                                                indexer.runIndexer())
                                .withTimeout(Seconds.of(3))
                                .andThen(Commands.parallel(shooter.stopShooter(), indexer.runStopIndexer()));
                Supplier<Command> LONG_shoot = () -> Commands
                                .parallel(shooter.runShooterAuto(() -> Constants.ShooterPreferences.LONG),
                                                indexer.runIndexer())
                                .withTimeout(Seconds.of(3))
                                .andThen(Commands.parallel(shooter.stopShooter(), indexer.runStopIndexer()));

                // Supplier<Command> quickShoot = () -> Commands.sequence(
                // shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG),
                // new AutoFire(shooter, indexer, hopper, () ->
                // ShooterPreferences.INDEXER_VELOCITY)
                // .repeatedly())
                // .withTimeout(3)
                // .andThen(Commands.sequence(
                // indexer.runStopIndexer(),
                // shooter.stopShooter(),
                // indexer.runStopIndexer()));

                // Supplier<Command> midShoot = () -> Commands.sequence(
                // shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG),
                // new AutoFire(shooter, indexer, hopper, () ->
                // ShooterPreferences.INDEXER_VELOCITY)
                // .repeatedly())
                // .withTimeout(3)
                // .andThen(Commands.sequence(
                // indexer.runStopIndexer(),
                // shooter.stopShooter(),
                // indexer.runStopIndexer()));

                // Supplier<Command> fullShoot = () -> Commands.sequence(
                // shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG),
                // new AutoFire(shooter, indexer, hopper, () ->
                // ShooterPreferences.INDEXER_VELOCITY)
                // .repeatedly())
                // .withTimeout(7)
                // .andThen(Commands.sequence(
                // indexer.runStopIndexer(),
                // shooter.stopShooter(),
                // indexer.runStopIndexer()));

                /* Full Autos */ // TODO: DON'T FORGET THE COMMAS
                // Test/Move to a position
                // Command blueCenter = new SequentialCommandGroup(
                // // MoveTo_blueCenter.get()
                // );
                Command FrontHubShoot = new SequentialCommandGroup(
                                MoveTo_FrontHubShoot.get(),
                                _shoot.get());
                Command blueCenterToDepot = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_depotFaceIn.get());

                Command TEST = new SequentialCommandGroup(
                                MoveTo_IntakeIN_FrontHubShoot.get(),
                                _shoot.get());
                Command JUSTSHOOT = new SequentialCommandGroup(
                                // MoveTo_blueCenter.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                MEDIUM_shoot.get()
                // MoveTo_rightBump_AllianceToFieldStart.get(),
                // MoveTo_rightBump_AllianceToFieldEnd.get(),
                // MoveTo_centerRightIntakeStart.get(),
                // MoveTo_centerRightIntakeEnd.get(),
                // MoveTo_leftLoadInZone.get()
                // MoveTo_rightLoadInZone.get()
                );
                // Mini Autos
                Command climbCommand = climber.runClimbUp().withName("Auto Climb Up");
                SmartDashboard.putData(climbCommand);
                // Depot
                Command DepotFaceIn = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get());
                // Bumps
                Command rightBumpToField = new SequentialCommandGroup(
                                // MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get());

                Command leftBumpToField = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get());

                Command rightBumpToAlliance = new SequentialCommandGroup(
                                // MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get());

                Command leftBumpToAlliance = new SequentialCommandGroup(
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get());
                // Center Harvest
                Command CenterHarvest = new SequentialCommandGroup(
                                // MoveTo_centerRightIntakeStart.get(),
                                // MoveTo_centerRightIntakeEnd.get()
                                MoveTo_rightLoadInZone.get());
                Command CenterRtoLSupplying = new SequentialCommandGroup(
                                MoveTo_leftLoadInZone.get(),
                                MoveTo_rightLoadInZone.get());

                Command climb = new SequentialCommandGroup(
                                MoveTo_preClimbPosition.get(),
                                climbCommand,
                                MoveTo_climbPosition.get(),
                                climber.runClimbDown(),
                                Commands.print("climbing"));

                // Actual Full autos
                Command LeftStartDepotScore = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get());

                Command RightStartDepotScore = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_offBlueCenter1.get(),
                                MoveTo_offBlueCenter2.get(),
                                MoveTo_rightOfDepotFaceOut.get(),
                                MoveTo_midOfDepotFaceOut.get(),
                                MoveTo_depotFaceOut.get(),
                                MoveTo_leftOfDepotFaceOut.get(),
                                MoveTo_FrontHubShoot.get());
                Command cmd = climber.runExtendClimber().withName("Auto Climb");
                SmartDashboard.putData(cmd);
                // Commands.parallel(MoveTo_fieldSideLeftBump.get(), cmd));
                // Commands.print("climbing").andThen(Commands.waitSeconds(3))

                Command CenterStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // _shoot.get(),
                                MoveTo_feedingStation_centerStart.get(),
                                Commands.parallel(intake.runIntakeOut(), MoveTo_feedingStation.get()),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                MEDIUM_shoot.get());

                Command LeftStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // _shoot.get(),
                                MoveTo_feedingStation_leftStart.get(),
                                Commands.parallel(intake.runIntakeOut(), MoveTo_feedingStation.get()),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                MEDIUM_shoot.get());

                Command RightStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // _shoot.get(),
                                MoveTo_feedingStation.get(),
                                Commands.parallel(intake.runIntakeOut(), MoveTo_feedingStation.get()),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                MEDIUM_shoot.get());

                // TODO: Fix this
                Command RightStartCenterHarvestInLeft = new SequentialCommandGroup(
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerRightIntakeStart.get(),
                                MoveToPickup_centerRightIntakeEnd.get(),
                                MoveToIntakeUp_centerRightIntakeEndLookHub.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_FrontHubShoot.get()// ,
                // MEDIUM_shoot.get()
                );

                Command LeftStartCenterHarvestInRight = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerLeftIntakeStart.get(),
                                MoveToPickup_centerLeftIntakeEnd.get(),
                                MoveToIntakeUp_centerLeftIntakeEndLookHub.get(),
                                MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_FrontHubShoot.get()// ,
                // MEDIUM_shoot.get()
                );

                Command RightQuad = new SequentialCommandGroup(
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerRightIntakeStart.get(),
                                MoveToPickup_quadLeft.get(),
                                MoveToIntakeUp_quadLeft.get(),
                                MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                _shoot.get());

                Command LeftQuad = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerLeftIntakeStart.get(),
                                MoveToPickup_quadRight.get(),
                                MoveToIntakeUp_quadRight.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                _shoot.get());

                // Actual name: RightFeedShootCenterHarvest
                Command Right_Yum_Middle = new SequentialCommandGroup(
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_FrontHubShoot.get(),
                                _shoot.get(),
                                MoveTo_rightBump_AllianceToFieldStartDos.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerRightIntakeStart.get(),
                                MoveToPickup_centerRightIntakeEnd.get(),
                                MoveToIntakeUp_centerRightIntakeEndLookHub.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_FrontHubShoot.get());
                // TODO: Debug SelectedShootPOS
                Command LeftDepotShootCenterHarvestInLeftShoot = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get(),
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                MoveTo_centerRightIntakeStart.get(),
                                MoveTo_centerRightIntakeEnd.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_FrontHubShoot.get());

                Command TheShowboater = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get(),
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_FrontHubShoot.get());

                /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                        // Elastic
                // NamedCommands.registerCommand("blueCenter", blueCenter);
                NamedCommands.registerCommand("FrontHubShoot", FrontHubShoot);
                NamedCommands.registerCommand("blueCenterToDepot", blueCenterToDepot);
                NamedCommands.registerCommand("TEST", TEST);
                NamedCommands.registerCommand("DepotFaceIn", DepotFaceIn);
                NamedCommands.registerCommand("climb", climb);
                NamedCommands.registerCommand("JUSTSHOOT", JUSTSHOOT);

                NamedCommands.registerCommand("rightBumpToField", rightBumpToField);
                NamedCommands.registerCommand("leftBumpToField", leftBumpToField);
                NamedCommands.registerCommand("rightBumpToAlliance", rightBumpToAlliance);
                NamedCommands.registerCommand("leftBumpToAlliance", leftBumpToAlliance);

                NamedCommands.registerCommand("CenterHarvest", CenterHarvest);
                NamedCommands.registerCommand("LeftStartDepotScore", LeftStartDepotScore);
                NamedCommands.registerCommand("RightStartDepotScore", RightStartDepotScore);

                // Feeding station
                NamedCommands.registerCommand("RightStartFeedingStationScore", RightStartFeedingStationScore);
                NamedCommands.registerCommand("LeftStartFeedingStationScore", LeftStartFeedingStationScore);

                NamedCommands.registerCommand("RightStartCenterHarvestInLeft", RightStartCenterHarvestInLeft);
                NamedCommands.registerCommand("LeftStartCenterHarvestInRight", LeftStartCenterHarvestInRight);
                NamedCommands.registerCommand("Right_Yum_Middle",
                                Right_Yum_Middle);
                NamedCommands.registerCommand("LeftDepotShootCenterHarvestInLeftShoot",
                                LeftDepotShootCenterHarvestInLeftShoot);
                NamedCommands.registerCommand("TheShowboater", TheShowboater);
                NamedCommands.registerCommand("RightQuad", RightQuad);
                NamedCommands.registerCommand("LeftQuad", LeftQuad);
        }
}
