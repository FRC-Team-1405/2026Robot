package frc.robot.commands.AutoPilot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPilotCommands {

    // Rotations
    // Positive rotations are CCW:
    // https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
    public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
    public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);

    // Poses
    public static Supplier<Pose2d> pos1 = () -> new Pose2d(3.23, 2.4, Rotation2d.fromDegrees(90));

    public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
        /* Commands */
        // Uses command suppliers instead of commands so that we can reuse the same
        // command in an autonomous
        Supplier<Command> MoveTo_pos1 = () -> new AutoPilotCommand(
                () -> pos1.get(), drivetrain, "MoveTo_pos1");

        /* Full Autos */
        Command AP_auto1 = new SequentialCommandGroup(
                MoveTo_pos1.get());

        /* Register Commands */
        NamedCommands.registerCommand("AP_auto1", AP_auto1);
    }
}
