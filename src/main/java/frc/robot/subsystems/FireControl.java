package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class FireControl {
    public FireControl() {
    }
    
    //Finds the target angle off of the robot position and the target position 
    public static Rotation2d getTargetRotation(Pose2d robotPose, Pose2d targetPose) {
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d targetAngle = toTarget.getAngle();
        Rotation2d relativeAngle = targetAngle.minus(robotPose.getRotation());

        return relativeAngle;
    }
    
    
    public void periodic() { //TODO
        Rotation2d currentTarget = getTargetRotation(null, HUB_RED);
    }   
}