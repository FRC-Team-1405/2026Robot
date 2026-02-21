package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
 * The current button mapping
 *
 * A: brake
 * B: compass rotation mode. Robot faces the way the right stick is facing.
 * X: snake rotation mode. Robot faces the way it is moving / the left stick is
 * facing.
 * Y: standard rotation mode
 * 
 * Left Trigger: slow mode (25% of max speed)
 * Right Trigger: fast mode (100% of max speed)
 * No trigger pressed: normal speed mode (50% of max speed)
 * 
 * 
 * 
 * Start + Back (simultaneously): Zeroizes the robot
 * 
 * L/R bumper: nothing
 * D-pad: nothing
 * Press L/R sticks: nothing
 */

/**
 * Manages how the robot moves at certain speeds and rotates with modes. Modes
 * are changed with button input defined in RobotContainer.java.
 * 
 * @author Dylan Wilson
 */
public class MoveMode {
    /** The speed modes the robot can be in. */
    public enum Speed {
        SLOW,
        NORMAL,
        FAST;

        @Override
        public String toString() {
            switch (this) {
                case SLOW:
                    return "Slow";
                case NORMAL:
                    return "Normal";
                case FAST:
                    return "Fast";
                default:
                    return "error Mode$Speed.toString()";
            }
        }
    }

    /** The rotation modes the robot can be in. */
    public enum Rotation {
        STANDARD,
        SNAKE,
        COMPASS;

        @Override
        public String toString() {
            switch (this) {
                case STANDARD:
                    return "Standard";
                case SNAKE:
                    return "Snake";
                case COMPASS:
                    return "Compass";
                default:
                    return "error Mode$Rotation.toString()";
            }
        }
    }

    /** The speed mode that the robot is currently in. */
    private Speed currentSpeedMode = Speed.NORMAL;

    /** The rotation mode that the robot is currently in. */
    private Rotation currentRotationMode = Rotation.STANDARD;

    // TODO: Tune rotationController for physical robot
    private final PIDController rotationController = new PIDController(9, 0, 0.3);
    // sim: P=9, I=0, D=0.3

    private StringPublisher speedModePublisher; // For Eclipse
    private StringPublisher rotationModePublisher; // For Eclipse

    public MoveMode() {
        elasticInit();
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the currentSpeedMode to the speed parameter. Called in
     * RobotContainer.java
     * 
     * @param speed the speed to set to
     * @return a command that sets the robots speed mode
     */
    public InstantCommand setTo(final Speed speed) {
        return new InstantCommand(() -> {
            currentSpeedMode = speed;
            elasticUpdate();
        });
    }

    /**
     * Sets the currentRotationMode to the rotation parameter. Called in
     * RobotContainer.java
     * 
     * @param speed the rotation to set to
     * @return a command that sets the robots rotation mode
     */
    public InstantCommand setTo(final Rotation rotation) {
        return new InstantCommand(() -> {
            currentRotationMode = rotation;
            elasticUpdate();
        });
    }

    /**
     * Selects the speed mode to perform based on which Mode currentSpeedMode is
     * equal to. A parent method for all the other speed modes.
     * 
     * @return a DoubleSupplier that returns a multiplier of the robots max speed
     */
    public DoubleSupplier selectSpeedMode() {
        return () -> {
            return switch (currentSpeedMode) {
                case SLOW -> 0.25d;
                case NORMAL -> 0.5d;
                case FAST -> 1.0d;
            };
        };
    }

    /**
     * Selects the rotation mode to perform based on which Mode currentRotationMode
     * is equal to. A parent method for all the other rotation modes.
     * 
     * @param joystick       the connected Xbox Controller (for all rotation modes)
     * @param drivetrain     the Swerve Drivetrain (for SNAKE, COMPASS)
     * @param maxAngularRate rotational rate cap (for STANDARD)
     * 
     * @return a DoubleSupplier that returns a value ranging from -Math.PI to
     *         Math.PI radians, setting the angle of the robot
     */
    public DoubleSupplier selectRotationMode(final CommandXboxController joystick,
            final CommandSwerveDrivetrain drivetrain,
            final double maxAngularRate) {
        return () -> {
            return switch (currentRotationMode) {
                case STANDARD -> standardMode(joystick, maxAngularRate);
                case SNAKE -> snakeMode(joystick, drivetrain);
                case COMPASS -> compassMode(joystick, drivetrain);
            };
        };
    }

    /**
     * Standard mode is a rotation mode where the robot's angle is determined from
     * the right stick.
     * The angle adds onto or subracts from the value that the right stick is
     * giving, not directly setting the robot's angle to the angle the right stick
     * points--see CompassMode for that.
     * <p>
     * Full right on the right stick means strongest clockwise rotation,
     * while full left on the right stick means strongest counterclockwise rotation.
     * 
     * @param joystick       the connected Xbox Controller
     * @param maxAngularRate the final double that caps the robot's rotation rate,
     *                       defined in RobotContainer.java
     * @return the right stick's value capped by maxAngularRate
     */
    private double standardMode(final CommandXboxController joystick, final double maxAngularRate) {
        return -joystick.getRightX() * maxAngularRate;
    }

    /**
     * Snake mode is a rotation mode where the robot points in the direction that it
     * is driving in / the direction that the left stick is facing.
     * 
     * @param joystick   the connected Xbox Controller
     * @param drivetrain the Swerve Drivetrain
     * @return a difference in goal angle and current angle calculated using a
     *         PIDController
     */
    private double snakeMode(final CommandXboxController joystick, final CommandSwerveDrivetrain drivetrain) {
        final double joystickX = joystick.getLeftX();
        final double joystickY = joystick.getLeftY();

        return calculateForJoystick(joystickX, joystickY, drivetrain);
    }

    /**
     * Compass mode is a rotation mode where the robot points in the direction that
     * the right stick is facing.
     * 
     * @param joystick the connected Xbox Controller
     * @return a difference in goal angle and current angle calculated using a
     *         PIDController
     */
    private double compassMode(final CommandXboxController joystick, final CommandSwerveDrivetrain drivetrain) {
        final double joystickX = joystick.getRightX();
        final double joystickY = joystick.getRightY();

        return calculateForJoystick(joystickX, joystickY, drivetrain);
    }

    /**
     * Helper method for snakeMode() and compassMode() that calculates a difference
     * in a goal angle(given by joystick direction) and a current angle(given by
     * robot / drivetrain direction) from a PIDController. The only difference in
     * snakeMode() and compassMode() is that snakeMode() uses the left stick while
     * compassMode() uses the right.
     * 
     * @param joystickX  X-value from joystick
     * @param joystickY  Y-value form joystick
     * @param drivetrain the Swerve Drivetrain
     * @return a difference in goal angle and current angle calculated using a
     *         PIDController
     */
    private double calculateForJoystick(final double joystickX, final double joystickY,
            final CommandSwerveDrivetrain drivetrain) {
        // If no input from joystick, don't change angle
        if (joystickX == 0f && joystickY == 0f) {
            return 0;
        }

        double goalAngle = -Math.atan2(joystickY, joystickX) - Math.PI / 2; // radians
        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();

        rotationController.setSetpoint(goalAngle);
        double calculate = rotationController.calculate(currentAngle);

        SmartDashboard.putNumber("goalAngle", goalAngle);
        SmartDashboard.putNumber("currentAngle", currentAngle);

        return calculate;
    }

    /**
     * Manages Elastic display initialization for the currentMode.
     */
    private void elasticInit() {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("DataTable");

        final StringTopic speedModeTopic = table.getStringTopic("Speed Mode");
        speedModePublisher = speedModeTopic.publish();

        final StringTopic rotationModeTopic = table.getStringTopic("Rotation Mode");
        rotationModePublisher = rotationModeTopic.publish();

        elasticUpdate();
    }

    /**
     * Manages Elastic display whenever the currentSpeedMode, currentRotationMode or
     * isDisabled is changed.
     */
    private void elasticUpdate() {
        final String strSpeedMode = currentSpeedMode.toString();
        speedModePublisher.set(strSpeedMode);
        SmartDashboard.putString("Speed Mode", strSpeedMode);

        final String strRotationMode = currentRotationMode.toString();
        rotationModePublisher.set(strRotationMode);
        SmartDashboard.putString("Rotation Mode", strRotationMode);
    }
}