package frc.robot.util;

// import java.util.Optional;
// import java.util.function.Supplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;



public class FireControl {
    public FireControl() {
    }
    
    public static Rotation2d getTargetRotation(Pose2d robotPose, Pose2d targetPose) {
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d targetAngle = toTarget.getAngle();
        Rotation2d relativeAngle = targetAngle.minus(robotPose.getRotation());
        
        return relativeAngle;
    }
}
