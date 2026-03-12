package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static Pose2d BLUE_HUB = new Pose2d(4.546, 4.035, Rotation2d.kZero);

    // location of middle april tag on the blue hub
    public static Pose2d BLUE_HUB_EDGE = new Pose2d(4.0218614, 4.0346376, Rotation2d.kZero);

    // teams prefered shooting location close to hub, back a foot from the edge of
    // the hub from the front of the intake
    public static Pose2d BLUE_HUB_SHOOT_CLOSE = BLUE_HUB_EDGE.transformBy(new Transform2d(
            -RobotConstants.HALF_ROBOT_WIDTH - RobotConstants.INTAKE_EXTENSION_LENGTH - Units.inchesToMeters(12), 0,
            Rotation2d.kZero));
}