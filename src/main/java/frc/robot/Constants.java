package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final String CARNIVORE_BUS_NAME = "Sonic";
  public static final AprilTagFieldLayout apriltagLayout;
  public static final Translation2d fieldSize;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
      fieldSize = new Translation2d(apriltagLayout.getFieldLength(), apriltagLayout.getFieldWidth());
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Distance X_CLEAR_OFFSET = Distance.ofBaseUnits(Math.abs(
                                                                              apriltagLayout.getTagPose(12).get().getMeasureX().in(Meter)
                                                                              - apriltagLayout.getTagPose(13).get().getMeasureX().in(Meter)) / 2.0,
                                                                    Meter);

  public static final Distance Y_CLEAR_OFFSET = Distance.ofBaseUnits(Math.abs(
                                                                              apriltagLayout.getTagPose(7).get().getMeasureY().in(Meter)
                                                                              - apriltagLayout.getTagPose(8).get().getMeasureY().in(Meter)) / 2.0,
                                                                    Meter);

  public static final Pose2d HUB_RED = new Pose2d(apriltagLayout.getTagPose(8).get().getMeasureX(), apriltagLayout.getTagPose(9).get().getMeasureY(), new Rotation2d(0.0));
  public static final Pose2d HUB_BLUE = new Pose2d(apriltagLayout.getTagPose(27).get().getMeasureX(), apriltagLayout.getTagPose(26).get().getMeasureY(), new Rotation2d(0.0));

  public static final Pose2d RED_FEED_TOP = new Pose2d(apriltagLayout.getTagPose(7).get().getMeasureX().plus(X_CLEAR_OFFSET), apriltagLayout.getTagPose(7).get().getMeasureY().plus(Y_CLEAR_OFFSET), new Rotation2d(0.0));
  public static final Pose2d RED_FEED_BOT = new Pose2d(apriltagLayout.getTagPose(12).get().getMeasureX().plus(X_CLEAR_OFFSET), apriltagLayout.getTagPose(12).get().getMeasureY().minus(Y_CLEAR_OFFSET), new Rotation2d(0.0));
  
  public static final Pose2d BLUE_FEED_TOP = new Pose2d(apriltagLayout.getTagPose(28).get().getMeasureX().minus(X_CLEAR_OFFSET), apriltagLayout.getTagPose(28).get().getMeasureY().plus(Y_CLEAR_OFFSET), new Rotation2d(0.0));
  public static final Pose2d BLUE_FEED_BOT = new Pose2d(apriltagLayout.getTagPose(23).get().getMeasureX().minus(X_CLEAR_OFFSET), apriltagLayout.getTagPose(23).get().getMeasureY().minus(Y_CLEAR_OFFSET), new Rotation2d(0.0));
  
  /**
   * Annotate CAN ID fields with this annotation so we can detect duplicates in a
   * unit test
   */
  
  @Retention(RetentionPolicy.RUNTIME)
  @Target(ElementType.FIELD)
  public @interface CanId {
    /**
     * The type of device that this CAN ID is for.
     *
     * You can use the same CAN ID for two different devices of different types
     * (e.g.: a Spark MAX motor and a Spark FLEX motor, or a Spark MAX motor and an
     * encoder).
     * (This is because the real CAN ID is much larger, but WPILib gives us 6 bytes
     * for ID and uses the device ID
     * for the other bytes.)
     * We could be more specific than these types, but for now we expect to want to
     * use the same CAN ID for a motor
     * and a corresponding encoder, but not for two motors. This could change.
     */
    Type value();

    /**
     * The device types.
     */
    enum Type {
      MOTOR,
      ENCODER,
      PIGEON,
      PCM_CONTROLLER,
    }
  }

  public static final class LEDs {
    public static final int PWM_PIN = 0;
    public static final int LENGTH = 150;
  }

  public static final class Swerve {
    @CanId(CanId.Type.PIGEON)
    public static final int IMU_ID = 1;
    public static final double TELEOP_MAX_VELOCITY = 4.6;
    public static final double TELEOP_MAX_ACCELERATION = 5.5; // todo
    public static final double TELEOP_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(180);
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540);
    public static final double TELEOP_ANGLE_HOLD_FACTOR = 3.0;

    public static final class Odometry {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
      public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class PathFollowing {
      public static final PIDConstants TRANSLATION_CONSTANTS = 
        new PIDConstants(4.0, 0.0, 0.0);
      public static final PIDConstants ROTATION_CONSTANTS = 
        new PIDConstants(8.0,0.0, 0.8);
    }
    
    public static final class FrontLeftModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 14;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 10;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 10;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(12.375),
        Units.inchesToMeters(10.125)
          );
    }

    public static final class FrontRightModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 17;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 13;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(12.375),
          -Units.inchesToMeters(10.125));
    }

    public static final class BackLeftModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 15;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 11;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 11;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(12.375),
        Units.inchesToMeters(10.125));
    }

    public static final class BackRightModule {
      @CanId(CanId.Type.MOTOR)
      public static final int DRIVE_MOTOR_ID = 16;
      @CanId(CanId.Type.MOTOR)
      public static final int ROTATION_MOTOR_ID = 12;
      @CanId(CanId.Type.ENCODER)
      public static final int ROTATION_ENCODER_ID = 12;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(12.375),
          -Units.inchesToMeters(10.125));
    }
  }

}
