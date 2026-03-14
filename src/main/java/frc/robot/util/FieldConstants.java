package frc.robot.util;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.lib.AllianceSymmetry;

public class FieldConstants {
    public static Pose2d BLUE_HUB = new Pose2d(4.546, 4.035, Rotation2d.kZero);

    public static Pose2d BLUE_FEEDER_WALL = new Pose2d(0.0077469999999999995, 0.6659626, Rotation2d.kZero);

    public static Supplier<Pose2d> ALLIANCE_HUB_POSITION = () -> {
        return AllianceSymmetry.isBlue() ? BLUE_HUB : AllianceSymmetry.flip(BLUE_HUB);
    };

    // location of middle april tag on the blue hub
    public static Pose2d BLUE_HUB_EDGE = new Pose2d(4.0218614, 4.0346376, Rotation2d.kZero);

    // teams prefered shooting location close to hub, back a foot from the edge of
    // the hub from the front of the intake
    public static Pose2d BLUE_HUB_SHOOT_CLOSE = BLUE_HUB_EDGE.transformBy(new Transform2d(
            -RobotConstants.HALF_ROBOT_WIDTH - RobotConstants.INTAKE_EXTENSION_LENGTH - Units.inchesToMeters(12), 0,
            Rotation2d.kZero));

    public static Pose2d BLUE_FEED_ROBOT_POSITION = BLUE_FEEDER_WALL.transformBy(new Transform2d(
            RobotConstants.HALF_ROBOT_WIDTH + RobotConstants.INTAKE_EXTENSION_LENGTH + Units.inchesToMeters(1), 0.0,
            Rotation2d.k180deg));

    // Start Positions
    public static Pose2d CENTER_START = BLUE_HUB_EDGE.transformBy(new Transform2d(
            -RobotConstants.HALF_ROBOT_WIDTH, 0,
            Rotation2d.kZero));

    public static Pose2d LEFT_START = BLUE_HUB_EDGE.transformBy(new Transform2d(
            -RobotConstants.HALF_ROBOT_WIDTH, RobotConstants.ROBOT_WIDTH
                    * 2,
            Rotation2d.kZero));

    public static Pose2d RIGHT_START = BLUE_HUB_EDGE.transformBy(new Transform2d(
            -RobotConstants.HALF_ROBOT_WIDTH, -(RobotConstants.ROBOT_WIDTH * 2),
            Rotation2d.kZero));

    // Poses
        // off blue center only used for Right Start Depot Score
        public static Supplier<Pose2d> offBlueCenter1 = () -> new Pose2d(2, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> offBlueCenter2 = () -> new Pose2d(0.40, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> feedingStation = () -> FieldConstants.BLUE_FEED_ROBOT_POSITION;
        public static Supplier<Pose2d> rightLoadInZone = () -> new Pose2d(5.75, 2.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftLoadInZone = () -> new Pose2d(5.75, 5.5, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> quadLeft = () -> new Pose2d(7.5, 3.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> quadRight = () -> new Pose2d(7.5, 6.5, Rotation2d.fromDegrees(270));

        // Start Poses
        public static Supplier<Pose2d> startRightFaceIn = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> startLeftFaceIn = () -> new Pose2d(3.55, 7.65, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> startRightFaceFront = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(0));

        // Bump Poses
        // Left Bump
        private static double BUMP_RIGHT_FIELD_SIDE = 6.0;

        private static double BUMP_LEFT_ALLIANCE_SIDE = 3.3;
        public static Supplier<Pose2d> leftBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(270));

        public static Supplier<Pose2d> leftBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 5.5,
                        Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> leftBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 5.5,
                        Rotation2d.fromDegrees(180));
        // Right Bump
        public static Supplier<Pose2d> rightBump_AllianceToFieldStart = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(90));
        // TODO:Remove Dos
        public static Supplier<Pose2d> rightBump_AllianceToFieldStartDos = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> rightBump_AllianceToFieldEnd = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));

        public static Supplier<Pose2d> rightBump_FieldToAllianceStart = () -> new Pose2d(
                        BUMP_RIGHT_FIELD_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> rightBump_FieldToAllianceEnd = () -> new Pose2d(
                        BUMP_LEFT_ALLIANCE_SIDE, 2.5,
                        Rotation2d.fromDegrees(0));
        // TODO: fix this
        // Shooter Poses
        public static Supplier<Pose2d> FrontHubShoot = () -> new Pose2d(3.069361, 4.034638, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeIN_FrontHubShoot = () -> new Pose2d(2.069361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> IntakeOUT_FrontHubShoot = () -> new Pose2d(3.069361, 4.034638,
                        Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> RightHubShoot = () -> new Pose2d(3.069361, 3.034638, Rotation2d.fromDegrees(0));
        // Center Poses
        // y position to start center harvesting on the right side
        private static double RIGHT_START_HARVEST_HORIZONTAL_POINT = 1;
        // was 3 and 5.5
        private static double RIGHT_END_HARVEST_HORIZONTAL_POINT = 6;
        private static double LEFT_START_HARVEST_HORIZONTAL_POINT = 7;
        // was 3 and 5.5
        private static double LEFT_END_HARVEST_HORIZONTAL_POINT = 2;

        public static Supplier<Pose2d> farRightCenter = () -> new Pose2d(8.25,
                        RIGHT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> farLeftCenter = () -> new Pose2d(8.25,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(0));

        // centerRightIntakeStart was (7.75, 1, 90)
        public static Supplier<Pose2d> centerRightIntakeStart = () -> new Pose2d(7.45,
                        RIGHT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(90));
        // centerRightIntakeEnd was (7.75, 6, 90)
        public static Supplier<Pose2d> centerRightIntakeEnd = () -> new Pose2d(7.45,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(90));
        // rotation was 180
        public static Supplier<Pose2d> centerRightIntakeEndLookHub = () -> new Pose2d(7.75,
                        RIGHT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(180));
        // centerRightIntakeStart was (7.75, 7, 270)
        public static Supplier<Pose2d> centerLeftIntakeStart = () -> new Pose2d(7.75,
                        LEFT_START_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(270));
        // centerLeftIntakeEnd was (7.75, 2, 270)
        public static Supplier<Pose2d> centerLeftIntakeEnd = () -> new Pose2d(7.75,
                        LEFT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(270));

        // TODO:Fix lookHub
        public static Supplier<Pose2d> centerLeftIntakeEndLookHub = () -> new Pose2d(7.75,
                        LEFT_END_HARVEST_HORIZONTAL_POINT, Rotation2d.fromDegrees(180));

        // Depot Poses
        // wall pose is 0.40
        public static Supplier<Pose2d> depotFaceIn = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftOfDepotFaceIn = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> midOfDepotFaceIn = () -> new Pose2d(0.50, 6, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> rightOfDepotFaceIn = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> depotFaceOut = () -> new Pose2d(0.50, 6.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> leftOfDepotFaceOut = () -> new Pose2d(0.50, 7, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> midOfDepotFaceOut = () -> new Pose2d(0.50, 6, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> rightOfDepotFaceOut = () -> new Pose2d(0.50, 5.5, Rotation2d.fromDegrees(90));

        public static Supplier<Pose2d> LeftMidAlliance = () -> new Pose2d(2, 6, Rotation2d.kZero);

        // Constant Poses
        // Center Pose is 8,4, Blue Center Pose is 2,4,90
        public static Supplier<Pose2d> blueCenter = () -> new Pose2d(2, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> centerOfField = () -> new Pose2d(8, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> fourMeters = () -> new Pose2d(4, 0, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> preClimbPosition = () -> new Pose2d(2.5, 4, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> climbPosition = () -> new Pose2d(1.5, 4, Rotation2d.fromDegrees(180));
}