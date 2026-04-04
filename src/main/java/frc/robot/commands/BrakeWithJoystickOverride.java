package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MoveMode;
import frc.robot.subsystems.SwerveFeatures;

public class BrakeWithJoystickOverride extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final MoveMode moveMode;
    private final CommandXboxController joystick;
    private boolean previouslyStationary = false;
    private double stationaryThreshold = 0.1;

    private boolean brakeMode = false;

    private boolean previouslyHasJoystickInput = false;

    private static double DEADBAND = 0.1;

    private final SwerveRequest.FieldCentric teleopDriveRequest;

    public BrakeWithJoystickOverride(
            CommandSwerveDrivetrain drivetrain, MoveMode moveMode,
            CommandXboxController joystick) {
        this.drivetrain = drivetrain;
        this.moveMode = moveMode;
        this.joystick = joystick;

        teleopDriveRequest = SwerveFeatures.teleopDriveRequest(drivetrain, moveMode, joystick);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (isStationary() && !hasJoystickInput(joystick)) {
            if (!brakeMode) {
                brakeMode = true;
                drivetrain.idle();
                drivetrain.applyRequest(() -> SwerveFeatures.brake);
                System.out.println("BrakeWithJoystickOverride BRAKE");
                // CommandScheduler.getInstance().schedule(SwerveFeatures.brakeCommand(drivetrain));
            }
        } else {
            if (brakeMode) {
                brakeMode = false;
                System.out.println("BrakeWithJoystickOverride TELEOP");
                drivetrain.idle();
                drivetrain.applyRequest(() -> teleopDriveRequest);
                // CommandScheduler.getInstance().schedule(SwerveFeatures.teleopDriveCommand(drivetrain,
                // moveMode, joystick));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End BrakeWithJoystickOverride Command, interrupted: " + interrupted);
    }

    /**
     * check if the robot is stationary with a buffer so we don't have jitter
     * between stationary and non stationary states
     * 
     * @return
     */
    private boolean isStationary() {
        if (previouslyStationary) {
            stationaryThreshold = 0.2; // m/s
        } else {
            stationaryThreshold = 0.05; // m/s
        }
        // double stationaryThreshold = 0.1; // m/s

        double velocity = SwerveFeatures.getRobotVelocity(drivetrain);
        double angularVelocity = SwerveFeatures.getRobotAngularVelocity(drivetrain);
        boolean isStationary = velocity < stationaryThreshold && angularVelocity < stationaryThreshold;

        return isStationary;
    }

    private boolean hasJoystickInput(CommandXboxController joystick) {
        if (previouslyHasJoystickInput) {
            DEADBAND = 0.05;
        } else {
            DEADBAND = 0.2;
        }

        return Math.abs(joystick.getLeftX()) > DEADBAND ||
                Math.abs(joystick.getLeftY()) > DEADBAND ||
                Math.abs(joystick.getRightX()) > DEADBAND;
    }
}