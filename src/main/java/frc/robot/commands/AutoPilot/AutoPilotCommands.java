package frc.robot.commands.AutoPilot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.AprilTags;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPilotCommands {

        // Rotations
        // Positive rotations are CCW:
        // https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
        public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
        public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);
        // TODO: Test origin auto
        // Poses
        public static Supplier<Pose2d> pos1 = () -> AprilTags.getAprilTagPose(28);
        public static Supplier<Pose2d> examplePose = () -> new Pose2d(0, 0, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> blueCenter = () -> new Pose2d(2, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> rightBump = () -> new Pose2d(4, 2.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> depot = () -> new Pose2d(0.5, 6.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftOfDepot = () -> new Pose2d(0.5, 7, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> midOfDepot = () -> new Pose2d(0.5, 6, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> rightOfDepot = () -> new Pose2d(0.5, 5.5, Rotation2d.fromDegrees(180));
        // Center Pose is 8,4
        public static Supplier<Pose2d> centerOfField = () -> new Pose2d(8, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> origin = () -> new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
                /* Commands */
                // Uses command suppliers instead of commands so that we can reuse the same
                // command in an autonomous
                Supplier<Command> MoveTo_pos1 = () -> new AutoPilotCommand.Builder(
                                () -> pos1.get(), drivetrain, "MoveTo_pos1")
                                .withFlipPoseForAlliance(true)
                                .build();

                Supplier<Command> MoveTo_pos1_withTracking = () -> new AutoPilotCommand.Builder(
                                () -> pos1.get(), drivetrain, "MoveTo_pos1_withTracking")
                                .withPointTowardsDuringMotion(() -> AprilTags.getAprilTagPose(20)) // Example: track tag
                                                                                                   // 4 while moving
                                .withPointTowardsTransitionThreshold(0.7) // Transition to final rotation at 70% of the
                                                                          // path
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_blueCenter = () -> new AutoPilotCommand.Builder(
                                () -> blueCenter.get(), drivetrain, "MoveTo_blueCenter")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_origin = () -> new AutoPilotCommand.Builder(
                                () -> origin.get(), drivetrain, "MoveTo_origin")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_centerOfField = () -> new AutoPilotCommand.Builder(
                                () -> centerOfField.get(), drivetrain, "MoveTo_centerOfField")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depot = () -> new AutoPilotCommand.Builder(
                                () -> depot.get(), drivetrain, "MoveTo_depot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepot = () -> new AutoPilotCommand.Builder(
                                () -> leftOfDepot.get(), drivetrain, "MoveTo_leftOfDepot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepot = () -> new AutoPilotCommand.Builder(
                                () -> midOfDepot.get(), drivetrain, "MoveTo_midOfDepot")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepot = () -> new AutoPilotCommand.Builder(
                                () -> rightOfDepot.get(), drivetrain, "MoveTo_rightOfDepot")
                                .withFlipPoseForAlliance(true)
                                .build();

                // right from the driver station view
                Supplier<Command> MoveTo_rightBump = () -> new AutoPilotCommand.Builder(
                                () -> rightBump.get(), drivetrain, "MoveTo_rightBump")
                                .withFlipPoseForAlliance(true)
                                .build();

                /* Full Autos */ // TODO: DON'T FORGET THE COMMAS
                Command AP_auto1 = new SequentialCommandGroup(
                                MoveTo_pos1.get());

                Command AP_tracking_example = new SequentialCommandGroup(
                                MoveTo_pos1_withTracking.get());

                Command AP_blueCenter = new SequentialCommandGroup(
                                MoveTo_blueCenter.get());

                Command AP_origin = new SequentialCommandGroup(
                                MoveTo_origin.get());

                Command AP_blueCenterToDepot = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_depot.get());

                Command AP_blueCenterToOriginToRightBump = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_origin.get(),
                                MoveTo_rightBump.get());

                Command AP_blueScoreBumpFeed = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_rightBump.get(),
                                MoveTo_centerOfField.get());

                Command AP_FEEDME = new SequentialCommandGroup(
                                MoveTo_leftOfDepot.get(),
                                MoveTo_depot.get(),
                                MoveTo_midOfDepot.get(),
                                MoveTo_rightOfDepot.get());

                /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                        // Elastic
                NamedCommands.registerCommand("AP_auto1", AP_auto1);
                NamedCommands.registerCommand("AP_tracking_example", AP_tracking_example);
                NamedCommands.registerCommand("AP_blueCenter", AP_blueCenter);
                NamedCommands.registerCommand("AP_blueCenterToDepot", AP_blueCenterToDepot);
                NamedCommands.registerCommand("AP_blueCenterToOriginToRightBump", AP_blueCenterToOriginToRightBump);
                NamedCommands.registerCommand("AP_blueScoreBumpFeed", AP_blueScoreBumpFeed);
                NamedCommands.registerCommand("AP_FEEDME", AP_FEEDME);
                NamedCommands.registerCommand("AP_origin", AP_origin);
        }
}
