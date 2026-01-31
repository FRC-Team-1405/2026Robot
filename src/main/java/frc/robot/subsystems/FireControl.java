package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotPoseLookup;



public class FireControl {
    private Supplier<Pose2d> robotSupplier;
    private Supplier<Alliance> allianceSupplier;
    private Swerve swerveModule;
    private ArrayList<Pose2d> redList = new ArrayList<>(2);
    private ArrayList<Pose2d> blueList = new ArrayList<>(2);
    private Rotation2d currentTarget;
    private double distanceFromTarget;
    private Pose2d target;
    private ChassisSpeeds currentChassisSpeed;

    public FireControl(Supplier<Pose2d> robSupplier, Supplier<Alliance> allSupplier) {
        robotSupplier = robSupplier;
        allianceSupplier = allSupplier;
        blueList.add(Constants.BLUE_FEED_BOT); blueList.add(Constants.BLUE_FEED_TOP);
        redList.add(Constants.RED_FEED_BOT); redList.add(Constants.RED_FEED_TOP);
    }
    
    //Finds the target angle off of the robot position and the target position 
    public static Rotation2d getTargetRotation(Pose2d robotPose, Pose2d targetPose) {
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d targetAngle = toTarget.getAngle();
        Rotation2d relativeAngle = targetAngle.minus(robotPose.getRotation());

        return relativeAngle;
    }
    /**
     * @return 
     */
    private ArrayList<Pose2d> getValidTargets() {
        ArrayList<Pose2d> validTargets = new ArrayList<>();
        if (allianceSupplier.get() == Alliance.Blue) {
            if(Constants.BLUE_ALLIANCE_ZONE.contains(robotSupplier.get().getTranslation())) {
                validTargets.add(Constants.HUB_BLUE);
            } else {
                validTargets = blueList;
            }
        } else if (allianceSupplier.get() == Alliance.Red) {
            if(Constants.RED_ALLIANCE_ZONE.contains(robotSupplier.get().getTranslation())) {
                validTargets.add(Constants.HUB_RED);
            } else {
               validTargets = redList;
            }
        }
        return validTargets;
    }
    
    /**
     * @param robotPose
     * @return TargetLocation with lowest distance to robot
     */
    // RobotPose - TargetLocation = Distance to location
    public Pose2d getClosestTarget(Pose2d robotPose) { //TODO
        ArrayList<Pose2d> targets = getValidTargets();
        Pose2d closestTarget = null;
        double smallestValue = 1000.0;
        for (Pose2d p: targets) {
            double d = getDistance(p, robotPose);
            if (d < smallestValue) {
                smallestValue = d;
                closestTarget = p;
            }
        }
        return closestTarget;
    }
    
    private double getDistance(Pose2d x, Pose2d y) {
        double d = x.getTranslation().getDistance(y.getTranslation());
        return d;
    } 

    //Checks every cycle for the correct target loctation, distance, and robot sped
    public void periodic() { 
        target = getClosestTarget(robotSupplier.get());
        currentTarget = getTargetRotation(robotSupplier.get(), target);
        distanceFromTarget = getDistance(target, robotSupplier.get());
        currentChassisSpeed = swerveModule.getCurrentSpeeds();
    }   

    public double getOffset() { //TODO
        double offset;
        offset = 0.0;
        return offset;
    }

    public Rotation2d getCurrentTarget() {
        return currentTarget;
    }

    public double getDistanceFromTarget() {
        return distanceFromTarget;
    }

    public Pose2d getTargetPose() {
        return target;
    }

} 