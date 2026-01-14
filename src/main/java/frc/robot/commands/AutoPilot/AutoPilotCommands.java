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

    // Poses
    public static Supplier<Pose2d> pos1 = () -> AprilTags.getAprilTagPose(28);

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
                .withPointTowardsDuringMotion(() -> AprilTags.getAprilTagPose(20)) // Example: track tag 4 while moving
                .withPointTowardsTransitionThreshold(0.7) // Transition to final rotation at 70% of the path
                .withFlipPoseForAlliance(true)
                .build();

        /* Full Autos */
        Command AP_auto1 = new SequentialCommandGroup(
                MoveTo_pos1.get());

        Command AP_tracking_example = new SequentialCommandGroup(
                MoveTo_pos1_withTracking.get());

        /* Register Commands */
        NamedCommands.registerCommand("AP_auto1", AP_auto1);
        NamedCommands.registerCommand("AP_tracking_example", AP_tracking_example);
    }
}
