package frc.robot.commands.AutoPilot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.therekrab.autopilot.APConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.DriveToHubDistance;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.AutoCommands;
import frc.robot.subsystems.AdjustableHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_XY_THRESHOLD;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_THETA_THRESHOLD;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_BEELINE_THRESHOLD;

public class CommandsForAutos {
        // TODO: integrate shooting positions dropdown into all autos, include the shoot
        // from distance in the dropdown
        // TODO: add a start position dropdown, integrate into all autos
        // TODO: integrate shooting positions dropdown into all autos, include the shoot
        // from distance in the dropdown
        // TODO: add a start position dropdown, integrate into all autos

        // Velocity is max speed overall/ in a sec, how much can position change
        // Acceleration in a second, how much can velocity change
        // Jerk is how fast it starts and stops

        public static Command OVERRIDE_AUTO_COMMAND = null;
        // #region CONSTRAINTS
        private static final APConstraints bumpConstraints = new APConstraints()
                        .withAcceleration(20) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(6)
                        .withJerk(200);

        // TODO: Adjust
        private static final APConstraints fullFieldConstraints = new APConstraints()
                        .withAcceleration(1.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(4.0)
                        .withJerk(67);

        private static final TrapezoidProfile.Constraints centerHarvestConstraint = new TrapezoidProfile.Constraints(
                        0.5, 0.5);
        // .withJerk(67.0);
        // #endregion

        // Rotations
        // Positive rotations are CCW:
        // https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
        public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
        public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);
        private static double BUMP_RIGHT_FIELD_SIDE = 6.0;
        private static double BUMP_LEFT_ALLIANCE_SIDE = 3.3;
        // Poses
        // off blue center only used for Right Start Depot Score

        public static Supplier<Pose2d> feedingStation = () -> FieldConstants.BLUE_FEED_ROBOT_POSITION;
        public static Supplier<Pose2d> rightLoadInZone = () -> new Pose2d(5.75, 2.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftLoadInZone = () -> new Pose2d(5.75, 5.5, Rotation2d.fromDegrees(180));

        // #region START Poses
        public static Supplier<Pose2d> startRightFaceIn = () -> new Pose2d(BUMP_LEFT_ALLIANCE_SIDE, 5,
                        Rotation2d.fromDegrees(0)); // was 3.55,
        // 0.37,
        // Rotation2d.fromDegrees(90)
        public static Supplier<Pose2d> startLeftFaceIn = () -> new Pose2d(3.55, 7.65, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> startRightFaceFront = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(0));
        // #endregion

        // #region BUMP Poses
        // Left Bump

        // Values for our test field at home, TODO update to the real field values
        private static double BUMP_CROSSING_ANGLE = 45;

        public static Supplier<Pose2d> leftBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(315)); // was 0
        public static Supplier<Pose2d> leftBump_AllianceToFieldStart_LOOK_HUB = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(270)); // TODO:Test
        public static Supplier<Pose2d> leftBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(225)); // was 0

        public static Supplier<Pose2d> leftBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(225)); // was 180
        public static Supplier<Pose2d> leftBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(315)); // was 180
        // Right Bump
        public static Supplier<Pose2d> rightBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5, // was 2.5
                        Rotation2d.fromDegrees(45)); // rotation was 0
        public static Supplier<Pose2d> rightBump_AllianceToFieldStart_LOOK_HUB = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5, // was 2.5
                        Rotation2d.fromDegrees(90)); // rotation was 0
        public static Supplier<Pose2d> rightBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5, // was 2.5
                        Rotation2d.fromDegrees(125)); // rotation was 0

        public static Supplier<Pose2d> rightBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5,
                        Rotation2d.fromDegrees(125)); // was 180
        public static Supplier<Pose2d> rightBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(45)); // was 180
        public static Supplier<Pose2d> rightBump_FieldToAllianceEndDos = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(90)); // was 180
        // #endregion

        // TODO: fix this
        // #region SHOOTER Poses
        public static Supplier<Pose2d> FrontHubShoot = () -> new Pose2d(3.069361, 4.034638, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeIN_FrontHubShoot = () -> new Pose2d(3.569361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeOUT_FrontHubShoot = () -> new Pose2d(3.469361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> RightHubShoot = () -> new Pose2d(3.069361, 3.034638, Rotation2d.fromDegrees(0));
        // #endregion

        // #region Center Harvest Poses
        // y position to start center harvesting on the right side
        // TODO: Change for actual field
        private static double RIGHT_START_HARVEST_HORIZONTAL_POINT = 3; // was 1
        private static double RIGHT_END_HARVEST_HORIZONTAL_POINT = 5.0; // was 6
        private static double LEFT_START_HARVEST_HORIZONTAL_POINT = 5.5; // was 7 //was 6 at Finney
        private static double LEFT_END_HARVEST_HORIZONTAL_POINT = 2.5; // was 2
        private static double LEFT_QUAD_HARVEST_END_POINT = 2.5; // was 7

        // centerRightIntakeStart was (7.75, 1, 90)
        public static Supplier<Pose2d> centerRightIntakeStart = () -> new Pose2d(7.45,
                        RIGHT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(90));
        // centerRightIntakeEnd was (7.75, 6, 90)
        public static Supplier<Pose2d> centerRightIntakeEnd = () -> new Pose2d(7.45, RIGHT_END_HARVEST_HORIZONTAL_POINT,
                        Rotation2d.fromDegrees(90));
        // centerRightIntakeStart was (7.75, 7, 270)
        public static Supplier<Pose2d> centerLeftIntakeStart = () -> new Pose2d(7.75,
                        LEFT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(270));
        // centerLeftIntakeEnd was (7.75, 2, 270)
        public static Supplier<Pose2d> centerLeftIntakeEnd = () -> new Pose2d(7.75, LEFT_END_HARVEST_HORIZONTAL_POINT,
                        Rotation2d.fromDegrees(270));
        // rotation was 180
        public static Supplier<Pose2d> centerRightIntakeEndLookHub = () -> new Pose2d(7.75,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(200));

        public static Supplier<Pose2d> centerLeftIntakeEndLookHub = () -> new Pose2d(7.75,
                        LEFT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(180));

        public static Supplier<Pose2d> quadRight = () -> new Pose2d(7.5, 3.5, Rotation2d.fromDegrees(90)); // was 7.5,
                                                                                                           // 3.5
        public static Supplier<Pose2d> quadLeft = () -> new Pose2d(7.5, LEFT_QUAD_HARVEST_END_POINT,
                        Rotation2d.fromDegrees(270)); // was 7.5,
        // // 6.5
        // #endregion

        // #region DEPOT Poses
        // wall pose is 0.40
        public static Supplier<Pose2d> depotFaceIn = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftOfDepotFaceIn = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> midOfDepotFaceIn = () -> new Pose2d(0.50, 6, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> rightOfDepotFaceIn = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> depotFaceOut = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> leftOfDepotFaceOut = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> rightOfDepotFaceOut = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(90));

        public static Supplier<Pose2d> depot_BackFace_Start = () -> new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> depot_BackFace_End = () -> new Pose2d(0.75, 5.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> depot_BackFace_End_Dos = () -> new Pose2d(0.50, 5.5,
                        Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> midOfDepotFaceOut = () -> new Pose2d(0.50, 5.75, Rotation2d.fromDegrees(180));

        public static Supplier<Pose2d> towerDodge_Start = () -> new Pose2d(2, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> towerDodge_End = () -> new Pose2d(0.40, 4.85, Rotation2d.fromDegrees(90));
        // #endregion

        public static Supplier<Pose2d> LeftMidAlliance = () -> new Pose2d(2, 6, Rotation2d.kZero);

        // #region FIELD CONSTANT Poses
        // Center Pose is 8,4, Blue Center Pose is 2,4,90
        public static Supplier<Pose2d> centerOfField = () -> new Pose2d(8, 4, Rotation2d.fromDegrees(0));
        // #endregion

        public static Supplier<Pose2d> fourMeters = () -> new Pose2d(2, 0, Rotation2d.fromDegrees(0));

        public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber,
                        Intake intake,
                        Hopper hopper,
                        Indexer indexer,
                        Shooter shooter,
                        AdjustableHood hood) {
                /* Commands */
                // Uses command suppliers instead of commands so that we can reuse the same
                // command in an autonomous
                Supplier<Command> MoveTo_allianceCenter = () -> new AutoPilotV2Command.Builder(
                                () -> FieldConstants.BLUE_HUB_SHOOT_CLOSE, drivetrain, "MoveTo_allianceCenter")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_LeftMidAlliance = () -> new AutoPilotV2Command.Builder(
                                () -> LeftMidAlliance.get(), drivetrain, "MoveTo_LeftMidAlliance")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerOfField = () -> new AutoPilotV2Command.Builder(
                                () -> centerOfField.get(), drivetrain, "MoveTo_centerOfField")
                                .withFlipPoseForAlliance(true)
                                .build();

                // #region Feeding station Movements
                Supplier<Command> MoveTo_feedingStation = () -> new AutoPilotV2Command.Builder(
                                () -> feedingStation.get(), drivetrain, "MoveTo_feedingStation")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_feedingStation_leftStart = () -> Commands.parallel(intake.runIntakeOut(),
                                Commands.sequence(MoveTo_allianceCenter.get(),
                                                MoveTo_feedingStation.get()));
                Supplier<Command> MoveTo_feedingStation_centerStart = () -> Commands.parallel(intake.runIntakeOut(),
                                Commands.sequence(
                                                MoveTo_allianceCenter.get(),
                                                MoveTo_feedingStation.get()));

                Supplier<Command> MoveTo_fourMeters = () -> new AutoPilotV2Command.Builder(
                                () -> fourMeters.get(), drivetrain, "MoveTo_fourMeters")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(fullFieldConstraints)
                                .build();
                // #endregion

                // Shooter
                Supplier<Command> MoveTo_FrontHubShoot = () -> new DriveToHubDistance(drivetrain,
                                FieldConstants.ALLIANCE_HUB_POSITION,
                                shooter.getDistanceFromSpeed());
                Supplier<Command> MoveTo_New_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                                () -> FrontHubShoot.get(), drivetrain, "MoveTo_New_FrontHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_requestedSpeedDistanceToHub = () -> new DriveToHubDistance(drivetrain,
                                FieldConstants.ALLIANCE_HUB_POSITION,
                                shooter.getDistanceFromSpeed());

                // Move To Shooting Positions
                Supplier<Command> MoveTo_ClosestShootingPosition_SHORT = () -> Commands.sequence(
                                new InstantCommand(() -> shooter
                                                .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.SHORT)),
                                MoveTo_requestedSpeedDistanceToHub.get());
                Supplier<Command> MoveTo_ClosestShootingPosition_MEDIUM = () -> Commands.sequence(
                                new InstantCommand(() -> shooter
                                                .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.MEDIUM)),
                                MoveTo_requestedSpeedDistanceToHub.get());
                Supplier<Command> MoveTo_ClosestShootingPosition_LONG = () -> Commands.sequence(
                                new InstantCommand(() -> shooter
                                                .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.LONG)),
                                MoveTo_requestedSpeedDistanceToHub.get());

                Supplier<Command> MoveTo_IntakeIN_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                                () -> IntakeIN_FrontHubShoot.get(), drivetrain, "MoveTo_IntakeIN_FrontHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_IntakeOUT_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                                () -> IntakeOUT_FrontHubShoot.get(), drivetrain, "MoveTo_IntakeOUT_FrontHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_RightHubShoot = () -> new AutoPilotV2Command.Builder(
                                () -> RightHubShoot.get(), drivetrain, "MoveTo_RightHubShoot")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Center Harvest(s)
                Supplier<Command> MoveTo_centerRightIntakeStart = () -> new AutoPilotV2Command.Builder(
                                () -> centerRightIntakeStart.get(), drivetrain, "MoveTo_centerRightIntakeStart")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerRightIntakeEnd = () -> new AutoPilotV2Command.Builder(
                                () -> centerRightIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                                .withFlipPoseForAlliance(true)
                                // TODO: .withConstraints(fullFieldConstraints)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeStart = () -> new AutoPilotV2Command.Builder(
                                () -> centerLeftIntakeStart.get(), drivetrain, "MoveTo_centerLeftIntakeStart")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeEnd = () -> new AutoPilotV2Command.Builder(
                                () -> centerLeftIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(centerHarvestConstraint)
                                .build();

                Supplier<Command> MoveAndDeploy_allianceCenter = () -> Commands.parallel(
                                MoveTo_allianceCenter.get(),
                                intake.runIntakeOut());

                Supplier<Command> MoveToPickup_centerLeftIntakeEnd = () -> Commands
                                .deadline(MoveTo_centerLeftIntakeEnd.get(), intake.runPickupIn());
                // deadline runs in
                // parrallel until the
                // first command
                // finishes
                Supplier<Command> MoveTo_quadRight = () -> new AutoPilotV2Command.Builder(
                                () -> quadRight.get(), drivetrain, "MoveTo_quadRight")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_quadLeft = () -> new AutoPilotV2Command.Builder(
                                () -> quadLeft.get(), drivetrain, "MoveTo_quadRight")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_centerRightIntakeEndLookHub = () -> new AutoPilotV2Command.Builder(
                                () -> centerRightIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                                .withFlipPoseForAlliance(true)
                                .withProfileThresholds(
                                                16.0, 5.0, DEFAULT_BEELINE_THRESHOLD)
                                // .withWaitSeconds(1)
                                .build();

                Supplier<Command> MoveTo_centerLeftIntakeEndLookHub = () -> new AutoPilotV2Command.Builder(
                                () -> centerLeftIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                                .withFlipPoseForAlliance(true)
                                // .withWaitSeconds(1)
                                .build();

                Supplier<Command> MoveToIntakeUp_centerLeftIntakeEndLookHub = () -> Commands
                                .parallel(MoveTo_centerLeftIntakeEndLookHub.get(), intake.runIntakeCenter());

                Supplier<Command> MoveTo_rightLoadInZone = () -> new AutoPilotV2Command.Builder(
                                () -> rightLoadInZone.get(), drivetrain, "MoveTo_rightLoadInZone")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftLoadInZone = () -> new AutoPilotV2Command.Builder(
                                () -> leftLoadInZone.get(), drivetrain, "MoveTo_leftLoadInZone")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Depot
                Supplier<Command> MoveTo_depotFaceIn = () -> new AutoPilotV2Command.Builder(
                                () -> depotFaceIn.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                                () -> leftOfDepotFaceIn.get(), drivetrain, "MoveTo_leftOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                                () -> midOfDepotFaceIn.get(), drivetrain, "MoveTo_midOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                                () -> rightOfDepotFaceIn.get(), drivetrain, "MoveTo_rightOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depotFaceOut = () -> new AutoPilotV2Command.Builder(
                                () -> depotFaceOut.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                                () -> leftOfDepotFaceOut.get(), drivetrain, "MoveTo_leftOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                                () -> midOfDepotFaceOut.get(), drivetrain, "MoveTo_midOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                                () -> rightOfDepotFaceOut.get(), drivetrain, "MoveTo_rightOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depot_BackFace_Start = () -> new AutoPilotV2Command.Builder(
                                () -> depot_BackFace_Start.get(), drivetrain, "MoveTo_depot_BackFace_Start")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depot_BackFace_End = () -> new AutoPilotV2Command.Builder(
                                () -> depot_BackFace_End.get(), drivetrain, "MoveTo_depot_BackFace_End")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depot_BackFace_End_Dos = () -> new AutoPilotV2Command.Builder(
                                () -> depot_BackFace_End_Dos.get(), drivetrain, "MoveTo_depot_BackFace_End_Dos")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_towerDodge_Start = () -> new AutoPilotV2Command.Builder(
                                () -> towerDodge_Start.get(), drivetrain, "MoveTo_towerDodge_Start")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_towerDodge_End = () -> new AutoPilotV2Command.Builder(
                                () -> towerDodge_End.get(), drivetrain, "MoveTo_towerDodge_End")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_selectedShootPosition = () -> new AutoPilotV2Command.Builder(
                                AutoCommands.getShootPosition(), drivetrain, "MoveTo_selectedShootPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                // right from the driver station view
                // Bump Stuff

                Supplier<Command> MoveTo_leftBump_AllianceToFieldStart = () -> new AutoPilotV2Command.Builder(
                                () -> leftBump_AllianceToFieldStart.get(), drivetrain,
                                "MoveTo_leftBump_AllianceToFieldStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_leftBump_AllianceToFieldStart_LOOK_HUB = () -> new AutoPilotV2Command.Builder(
                                () -> leftBump_AllianceToFieldStart_LOOK_HUB.get(), drivetrain,
                                "MoveTo_leftBump_AllianceToFieldStart_LOOK_HUB")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_leftBump_AllianceToFieldEnd = () -> new AutoPilotV2Command.Builder(
                                () -> leftBump_AllianceToFieldEnd.get(), drivetrain,
                                "MoveTo_leftBump_AllianceToFieldEnd")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_rightBump_AllianceToFieldStart = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_AllianceToFieldStart.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_AllianceToFieldStart_LOOK_HUB.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                // Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new
                // AutoPilotV2Command.Builder(
                // () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
                // "MoveTo_rightBump_AllianceToFieldEnd")
                // .withFlipPoseForAlliance(true)
                // .build();

                Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
                                "MoveTo_rightBump_AllianceToFieldEnd")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_leftBump_FieldToAllianceStart = () -> new AutoPilotV2Command.Builder(
                                () -> leftBump_FieldToAllianceStart.get(), drivetrain,
                                "MoveTo_leftBump_FieldToAllianceStart")
                                .withFlipPoseForAlliance(true)
                                // .withConstraints(bumpConstraints)
                                .build();

                Supplier<Command> MoveTo_leftBump_FieldToAllianceEnd = () -> new AutoPilotV2Command.Builder(
                                () -> leftBump_FieldToAllianceEnd.get(), drivetrain,
                                "MoveTo_leftBump_FieldToAllianceEnd")
                                .withFlipPoseForAlliance(true)
                                .withProfileThresholds(6, 12, DEFAULT_BEELINE_THRESHOLD)
                                .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_rightBump_FieldToAllianceStart = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_FieldToAllianceStart.get(), drivetrain,
                                "MoveTo_rightBump_FieldToAllianceStart")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightBump_FieldToAllianceEnd = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_FieldToAllianceEnd.get(), drivetrain,
                                "MoveTo_rightBump_FieldToAllianceEnd")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightBump_FieldToAllianceEndDos = () -> new AutoPilotV2Command.Builder(
                                () -> rightBump_FieldToAllianceEndDos.get(), drivetrain,
                                "MoveTo_rightBump_FieldToAllianceEndDos")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_startRightFaceIn = () -> new AutoPilotV2Command.Builder(
                                () -> startRightFaceIn.get(), drivetrain,
                                "MoveTo_startRightFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
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

                Supplier<Command> mediumShoot = () -> Commands.sequence(
                                shooter.runSetRequestedSpeed(() -> ShooterPreferences.MEDIUM),
                                AutoFire.autonomous(shooter, indexer,
                                                () -> ShooterPreferences.INDEXER_VELOCITY))
                                .withTimeout(10)
                                .andThen(Commands.sequence(
                                                indexer.runStopIndexer(),
                                                shooter.stopShooter(),
                                                indexer.runStopIndexer()));

                Supplier<Command> shortShoot = () -> Commands.sequence(
                                shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT),
                                AutoFire.autonomous(shooter, indexer,
                                                () -> ShooterPreferences.INDEXER_VELOCITY))
                                .withTimeout(10)
                                .andThen(Commands.sequence(
                                                indexer.runStopIndexer(),
                                                shooter.stopShooter(),
                                                indexer.runStopIndexer()));
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

                Command FrontHubShoot = new SequentialCommandGroup(
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());
                Command LeftStart_ToDepot = new SequentialCommandGroup(
                                // Commands.parallel(MoveTo_leftBump_AllianceToFieldStart.get(),
                                // intake.runIntakeOut()),

                                Commands.parallel(intake.runIntakeOut(),
                                                MoveTo_leftBump_AllianceToFieldStart.get()),
                                MoveTo_depot_BackFace_Start.get(),
                                Commands.deadline(
                                                Commands.sequence(MoveTo_depot_BackFace_End.get(),
                                                                MoveTo_depot_BackFace_End_Dos.get(),
                                                                MoveTo_midOfDepotFaceOut.get()),
                                                intake.runPickupIn()),

                                MoveTo_depot_BackFace_Start.get(),

                                // intake.runPickupStop(),
                                MoveTo_New_FrontHubShoot.get());

                Command TEST = new SequentialCommandGroup(
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_IntakeIN_FrontHubShoot.get(),
                                // shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT),
                                // new SetHoodPosition(hood, HoodAngles.SHORT),
                                // new AutoFire(shooter, indexer, hopper, () ->
                                // ShooterPreferences.INDEXER_VELOCITY)

                                // Commands.waitSeconds(0.5),
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_IntakeOUT_FrontHubShoot.get(),
                                // intake.runIntakeOut()
                                // TODO: Add intaking stuff
                                MoveTo_depot_BackFace_Start.get(),
                                intake.runIntakeOut(),
                                MoveTo_depot_BackFace_End.get(),

                                MoveTo_depot_BackFace_Start.get(),
                                MoveTo_New_FrontHubShoot.get());
                Command JUSTSHOOT = new SequentialCommandGroup(
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                MoveTo_FrontHubShoot.get(),
                                intake.runIntakeOut(),
                                // intake.runPickupIn().withTimeout(0.5),
                                Commands.parallel(MoveTo_IntakeIN_FrontHubShoot.get(),
                                                intake.runPickupIn().withTimeout(3.0)),
                                intake.runIntakeCenter(),
                                // MoveTo_depot_BackFace_End.get(),
                                MoveTo_IntakeOUT_FrontHubShoot.get()

                // MoveTo_rightBump_AllianceToFieldStart.get(),
                // MoveTo_rightBump_AllianceToFieldEnd.get(),
                // MoveTo_centerRightIntakeStart.get(),
                // MoveTo_centerRightIntakeEnd.get(),
                // MoveTo_leftLoadInZone.get()
                // MoveTo_rightLoadInZone.get()
                );

                Command JUST_SHOOT_FROM_ANYWHERE = Commands.sequence(MoveTo_ClosestShootingPosition_SHORT.get(),
                                shortShoot.get());

                Command LeftStart_JUSTSHOOT = new SequentialCommandGroup(
                                MoveTo_LeftMidAlliance.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                mediumShoot.get());
                // Mini Autos

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
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.parallel(MoveTo_centerRightIntakeStart.get(), intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn()),
                                Commands.parallel(MoveTo_centerRightIntakeEndLookHub.get(), intake.runIntakeCenter()),
                                // MoveTo_centerRightIntakeEndLookHub.get(),
                                // intake.runIntakeCenter(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());
                Command CenterRtoLSupplying = new SequentialCommandGroup(
                                MoveTo_leftLoadInZone.get(),
                                MoveTo_rightLoadInZone.get());

                // Actual Full autos
                Command LeftStartDepotScore = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get());

                Command RightStartDepotScore = new SequentialCommandGroup(
                                MoveTo_allianceCenter.get(),
                                MoveTo_towerDodge_Start.get(),
                                MoveTo_towerDodge_End.get(),
                                MoveTo_rightOfDepotFaceOut.get(),
                                MoveTo_midOfDepotFaceOut.get(),
                                MoveTo_depotFaceOut.get(),
                                MoveTo_leftOfDepotFaceOut.get(),
                                MoveTo_FrontHubShoot.get());

                Command CenterStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                MoveTo_feedingStation_centerStart.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command LeftStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                MoveTo_feedingStation_leftStart.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command RightStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                Commands.parallel(intake.runIntakeOut(), MoveTo_feedingStation.get()),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                // TODO: Fix this
                Command RightStartCenterHarvestInLeft = new SequentialCommandGroup(
                                Commands.parallel(MoveTo_rightBump_AllianceToFieldEnd.get(), intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(), intake.runPickupIn()),
                                Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn()),
                                MoveTo_centerRightIntakeEndLookHub.get(),
                                Commands.deadline(MoveTo_leftBump_FieldToAllianceStart.get(), intake.runPickupIn()),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                mediumShoot.get());

                Command LeftStartCenterHarvestInRight = new SequentialCommandGroup(
                                Commands.parallel(MoveTo_leftBump_AllianceToFieldEnd.get(), intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerLeftIntakeStart.get(), intake.runPickupIn()),
                                Commands.deadline(MoveTo_centerLeftIntakeEnd.get(), intake.runPickupIn()),
                                MoveTo_centerLeftIntakeEndLookHub.get(),
                                Commands.deadline(MoveTo_rightBump_FieldToAllianceStart.get(), intake.runPickupIn()),
                                MoveTo_rightBump_FieldToAllianceEnd.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                mediumShoot.get());

                Command RightQuad = new SequentialCommandGroup(
                                Commands.parallel(MoveTo_rightBump_AllianceToFieldEnd.get(), intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(), intake.runPickupIn()),
                                Commands.deadline(MoveTo_quadRight.get(), intake.runPickupIn()),
                                Commands.deadline(MoveTo_rightBump_FieldToAllianceStart.get(), intake.runPickupIn()),
                                MoveTo_rightBump_FieldToAllianceEnd.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                mediumShoot.get());

                Command LeftQuad = new SequentialCommandGroup(
                                Commands.parallel(MoveTo_leftBump_AllianceToFieldEnd.get(), intake.runIntakeOut()),
                                
                                
                                Commands.deadline(Commands.sequence(MoveTo_centerLeftIntakeStart.get(),
                                MoveTo_quadLeft.get(), MoveTo_leftBump_FieldToAllianceStart.get()), intake.runPickupIn()),
                                
                                Commands.parallel(MoveTo_leftBump_FieldToAllianceEnd.get(), intake.runIntakeCenter()),
                                mediumShoot.get());

                // Actual name: RightFeedShootCenterHarvest
                Command Right_Yum_Middle = new SequentialCommandGroup(
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get(),
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(),
                                                intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn()),
                                Commands.parallel(MoveTo_centerRightIntakeEndLookHub.get(), intake.runIntakeCenter()),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_FrontHubShoot.get());

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
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get());

                Command fourMeters = new SequentialCommandGroup(
                                intake.runIntakeOut(),
                                Commands.parallel(MoveTo_fourMeters.get(), intake.runPickupIn()),
                                intake.runPickupStop());
                Command TheShowboater = new SequentialCommandGroup(
                                MoveTo_depot_BackFace_Start.get(),
                                MoveTo_depot_BackFace_End.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_New_FrontHubShoot.get());

                /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                        // Elastic
                // NamedCommands.registerCommand("blueCenter", blueCenter);
                NamedCommands.registerCommand("FrontHubShoot", FrontHubShoot);
                NamedCommands.registerCommand("blueCenterToDepot", LeftStart_ToDepot);
                NamedCommands.registerCommand("TEST", TEST);
                NamedCommands.registerCommand("DepotFaceIn", DepotFaceIn);
                NamedCommands.registerCommand("JUSTSHOOT", JUSTSHOOT);
                NamedCommands.registerCommand("LeftStart_JUSTSHOOT", LeftStart_JUSTSHOOT);

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
                NamedCommands.registerCommand("CenterStartFeedingStationScore",
                                CenterStartFeedingStationScore);
                NamedCommands.registerCommand("LeftDepotShootCenterHarvestInLeftShoot",
                                LeftDepotShootCenterHarvestInLeftShoot);
                NamedCommands.registerCommand("TheShowboater", TheShowboater);
                NamedCommands.registerCommand("RightQuad", RightQuad);
                NamedCommands.registerCommand("LeftQuad", LeftQuad);
                NamedCommands.registerCommand("fourMeters", fourMeters);

                // TODO: add window in Elastic
                OVERRIDE_AUTO_COMMAND = RightQuad;
        }
}
