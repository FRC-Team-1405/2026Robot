package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Manages how the robot rotates.
 * 
 * @author Dylan Wilson
 */
public class MoveMode {
    private enum Mode {
        STANDARD,
        SNAKE,
        COMPASS
    }

    /**
     * Sets the currentMode for the robot. Is a private Command class to
     * communicate with the RobotContainer.java's XboxController joystick.
     */
    private class MoveModeCommand extends InstantCommand {
        private Mode modeToSet;

        private MoveModeCommand(Mode modeToSet) {
            this.modeToSet = modeToSet;
        }

        @Override
        public void initialize() {
            currentMode = modeToSet;
            elasticUpdate();
        }
    }

    private static Mode currentMode = Mode.STANDARD;

    private static StringPublisher modePublisher; // For Eclipse
    // TODO: Tune rotationController for physical robot
    private final PIDController rotationController = new PIDController(12, 0, 0);
    private double goalAngle;
    private double currentAngle;
    private double calculate;

    public MoveMode() {
        elasticInit();
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Selects the Mode to perform based on which Mode currentMode is equal to. A
     * parent method for all the other modes.
     * 
     * @param joystick       the connected Xbox Controller (all modes)
     * @param drivetrain     the Swerve Drivetrain (SNAKE, COMPASS)
     * @param maxAngularRate rotational rate cap (STANDARD)
     */
    public double selectMode(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain,
            double maxAngularRate) {
        return switch (currentMode) {
            case STANDARD -> standardMode(joystick, maxAngularRate);
            case SNAKE -> snakeMode(joystick, drivetrain);
            case COMPASS -> compassMode(joystick, drivetrain);
        };
    }

    /**
     * Sets currentMode to Mode.STANDARD. Enabled by the X button.
     * <p>
     * 
     * @return a Command that sets currentMode to Mode.STANDARD
     */
    public Command setToStandardMode() {
        return new MoveModeCommand(Mode.STANDARD);
    }

    /**
     * Sets currentMode to Mode.SNAKE. Enabled by the Y button.
     * <p>
     * 
     * @return a Command that sets currentMode to Mode.SNAKE
     */
    public Command setToSnakeMode() {
        return new MoveModeCommand(Mode.SNAKE);
    }

    /**
     * Sets currentMode to Mode.COMPASS. Enabled by the B button.
     * <p>
     * 
     * @return a Command that sets currentMode to Mode.SNAKE
     */
    public Command setToCompassMode() {
        return new MoveModeCommand(Mode.COMPASS);
    }

    /**
     * Standard mode is where the robot's angle is determined from the right stick.
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
    private double standardMode(CommandXboxController joystick, double maxAngularRate) {
        return -joystick.getRightX() * maxAngularRate;
    }

    /**
     * Snake mode is where the robot points in the direction that it is driving in /
     * the direction that the left stick is facing.
     * 
     * @param joystick   the connected Xbox Controller
     * @param drivetrain the Swerve Drivetrain
     * @return a difference in goal angle and current angle calculated using a
     *         PIDController
     */
    private double snakeMode(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain) {
        double joystickX = joystick.getLeftX();
        double joystickY = joystick.getLeftY();

        return calculateForJoystick(joystickX, joystickY, drivetrain);
    }

    /**
     * Compass mode is where the robot points in the direction that the right stick
     * is facing.
     * 
     * @param joystick the connected Xbox Controller
     * @return a difference in goal angle and current angle calculated using a
     *         PIDController
     */
    private double compassMode(CommandXboxController joystick, CommandSwerveDrivetrain drivetrain) {
        double joystickX = joystick.getRightX();
        double joystickY = joystick.getRightY();

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
    private double calculateForJoystick(double joystickX, double joystickY, CommandSwerveDrivetrain drivetrain) {
        // If no input from joystick, don't change angle
        if (joystickX == 0f && joystickY == 0f) {
            return 0;
        }

        goalAngle = -Math.atan2(joystickY, joystickX) - Math.PI / 2; // radians
        currentAngle = drivetrain.getState().Pose.getRotation().getRadians();

        rotationController.setSetpoint(goalAngle);
        calculate = rotationController.calculate(currentAngle);

        return calculate;
    }

    /**
     * Manages Elastic display initialization for the currentMode.
     */
    private void elasticInit() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("DataTable");

        StringTopic modeTopic = table.getStringTopic("MoveMode");
        modePublisher = modeTopic.publish();

        elasticUpdate();
    }

    /**
     * Manages Elastic display whenever the currentMode is changed.
     */
    private void elasticUpdate() {
        String strMode = currentMode.toString();

        modePublisher.set(strMode);
        SmartDashboard.putString("Move Mode", strMode);
    }
}