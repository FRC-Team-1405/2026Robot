package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToHubDistance extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> hubPoseSupplier;
    private final Supplier<Double> desiredDistanceMeters;

    private Pose2d targetPose;

    private Command driveToPoseCommand;

    public DriveToHubDistance(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> hubPoseSupplier,
            Supplier<Double> desiredDistanceMeters) {

        this.drivetrain = drivetrain;
        this.hubPoseSupplier = hubPoseSupplier;
        this.desiredDistanceMeters = desiredDistanceMeters;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d hubPose = hubPoseSupplier.get();

        Translation2d robotToHub = hubPose.getTranslation().minus(robotPose.getTranslation());

        // Direction from robot → hub
        Translation2d direction = robotToHub.div(robotToHub.getNorm());

        // Compute the target point at the desired distance from the hub
        Translation2d targetTranslation = hubPose.getTranslation().minus(direction.times(desiredDistanceMeters.get()));

        Rotation2d targetRotation = hubPose.getTranslation()
                .minus(targetTranslation)
                .getAngle();

        targetPose = new Pose2d(targetTranslation, targetRotation);

        driveToPoseCommand = Commands.defer(() -> drivetrain.driveToPose(() -> Optional.of(targetPose), false),
                Set.of(drivetrain));
        driveToPoseCommand.initialize();
        // CommandScheduler.getInstance().schedule(driveToPoseCommand);

    }

    @Override
    public void execute() {
        driveToPoseCommand.execute();
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drivetrain.getState().Pose;
        double dist = current.getTranslation().getDistance(targetPose.getTranslation());
        return dist < 0.05; // 5 cm tolerance
    }

    @Override
    public void end(boolean interrupted) {
        if (driveToPoseCommand != null) {
            driveToPoseCommand.end(interrupted);
        }
    }
}