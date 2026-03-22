package frc.robot.commands.AutoPilot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.AllianceSymmetry;
import frc.robot.lib.FinneyCommand;
import frc.robot.lib.FinneyLogger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Simplified AutoPilot command — single control path, no theta controller
 * switching.
 *
 * <p>
 * Key differences from {@link AutoPilotCommand}:
 * <ul>
 * <li>Uses {@code FieldCentricFacingAngle} exclusively — no
 * {@code ApplyRobotSpeeds}, no manual frame conversion.</li>
 * <li>No distance-to-target code-path switching; every cycle takes the same
 * path.</li>
 * <li>Converts robot-relative speeds to field-relative before passing to
 * AutoPilot (the original command passed robot-relative speeds, which may
 * cause AutoPilot's internal motion profiler to miscalculate when the robot
 * is rotated).</li>
 * <li>Comprehensive per-module drive motor logging for oscillation
 * debugging.</li>
 * </ul>
 *
 * <pre>
 * // Basic usage
 * new AutoPilotV2Command.Builder(targetSupplier, drivetrain, "name").build();
 *
 * // With alliance flipping and custom heading PID
 * new AutoPilotV2Command.Builder(targetSupplier, drivetrain, "name")
 *         .withFlipPoseForAlliance(true)
 *         .withHeadingPID(1.5, 0.05)
 *         .build();
 * </pre>
 */
public class AutoPilotV2Command extends FinneyCommand {
    private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());
    private static final String NT = "APv2/";

    private final StructPublisher<Pose2d> targetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("SmartDashboard/Auto/APv2/TargetPose", Pose2d.struct).publish();

    // --- AutoPilot components ---
    private static final APConstraints kDefaultConstraints = new APConstraints()
            .withAcceleration(2.0)
            .withVelocity(2.0)
            .withJerk(10.0);

    private final APConstraints kConstraints;
    private final APProfile kProfile;
    private final Autopilot kAutopilot;

    // --- Instance state ---
    private APTarget m_target;
    private final Supplier<Pose2d> m_targetSupplier;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Optional<Rotation2d> m_entryAngle;
    private final boolean m_flipPoseForAlliance;
    private final String commandName;

    // Single swerve request for every cycle
    private final SwerveRequest.FieldCentricFacingAngle m_request;

    // Logging helpers
    private int m_cycleCount = 0;
    private static final int MODULE_LOG_INTERVAL = 5;

    // -----------------------------------------------------------------------
    // Builder
    // -----------------------------------------------------------------------

    public static class Builder {
        private final Supplier<Pose2d> targetSupplier;
        private final CommandSwerveDrivetrain drivetrain;
        private final String commandName;

        private Optional<Rotation2d> entryAngle = Optional.empty();
        private boolean flipPoseForAlliance = false;
        private APConstraints constraints = kDefaultConstraints;
        private double errorXYCentimeters = 2.0;
        private double errorThetaDegrees = 0.5;
        private double beelineRadiusCentimeters = 16.0;
        private double headingKP = 2.0;
        private double headingKD = 0.0;

        public Builder(Supplier<Pose2d> targetSupplier, CommandSwerveDrivetrain drivetrain, String commandName) {
            this.targetSupplier = targetSupplier;
            this.drivetrain = drivetrain;
            this.commandName = commandName;
        }

        public Builder withEntryAngle(Rotation2d entryAngle) {
            this.entryAngle = Optional.of(entryAngle);
            return this;
        }

        public Builder withFlipPoseForAlliance(boolean flip) {
            this.flipPoseForAlliance = flip;
            return this;
        }

        public Builder withConstraints(APConstraints constraints) {
            this.constraints = constraints;
            return this;
        }

        /**
         * @param errorXYCm       XY error threshold in centimeters
         * @param errorThetaDeg   theta error threshold in degrees
         * @param beelineRadiusCm beeline radius in centimeters
         */
        public Builder withProfileThresholds(double errorXYCm, double errorThetaDeg, double beelineRadiusCm) {
            this.errorXYCentimeters = errorXYCm;
            this.errorThetaDegrees = errorThetaDeg;
            this.beelineRadiusCentimeters = beelineRadiusCm;
            return this;
        }

        /**
         * Tune the heading PID used by {@code FieldCentricFacingAngle}.
         * Default is kP=2.0, kD=0.0.
         */
        public Builder withHeadingPID(double kP, double kD) {
            this.headingKP = kP;
            this.headingKD = kD;
            return this;
        }

        public AutoPilotV2Command build() {
            return new AutoPilotV2Command(this);
        }
    }

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    private AutoPilotV2Command(Builder b) {
        m_targetSupplier = b.targetSupplier;
        m_drivetrain = b.drivetrain;
        m_entryAngle = b.entryAngle;
        m_flipPoseForAlliance = b.flipPoseForAlliance;
        commandName = b.commandName;

        kConstraints = b.constraints;
        kProfile = new APProfile(kConstraints)
                .withErrorXY(Centimeters.of(b.errorXYCentimeters))
                .withErrorTheta(Degrees.of(b.errorThetaDegrees))
                .withBeelineRadius(Centimeters.of(b.beelineRadiusCentimeters));
        kAutopilot = new Autopilot(kProfile);

        m_request = new SwerveRequest.FieldCentricFacingAngle()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withHeadingPID(b.headingKP, 0, b.headingKD);

        addRequirements(m_drivetrain);
    }

    // -----------------------------------------------------------------------
    // Command lifecycle
    // -----------------------------------------------------------------------

    @Override
    public void initialize() {
        super.initialize();
        m_cycleCount = 0;

        if (m_flipPoseForAlliance
                && DriverStation.Alliance.Red.equals(DriverStation.getAlliance().orElse(null))) {
            m_target = new APTarget(AllianceSymmetry.flip(m_targetSupplier.get()));
        } else {
            m_target = new APTarget(m_targetSupplier.get());
        }

        targetPublisher.set(m_target.getReference());

        Pose2d currentPose = m_drivetrain.getState().Pose;
        double dist = currentPose.getTranslation().getDistance(m_target.getReference().getTranslation());

        fLogger.log("INIT %s | target(%.2f,%.2f,%.1f°) current(%.2f,%.2f,%.1f°) dist=%.2fm",
                getName(),
                m_target.getReference().getX(), m_target.getReference().getY(),
                m_target.getReference().getRotation().getDegrees(),
                currentPose.getX(), currentPose.getY(),
                currentPose.getRotation().getDegrees(),
                dist);
    }

    @Override
    public void execute() {
        m_cycleCount++;

        // --- 1. Read current state ---
        Pose2d pose = m_drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = m_drivetrain.getState().Speeds;

        // --- 2. Convert robot-relative speeds → field-relative ---
        // getState().Speeds is robot-relative.  AutoPilot plans in field coordinates,
        // so feeding robot-relative speeds causes its motion profiler to see the
        // velocity in the wrong direction when the robot is rotated — a likely
        // contributor to the oscillation bug in the original command.
        ChassisSpeeds fieldSpeeds = toFieldRelative(robotSpeeds, pose.getRotation());

        // --- 3. Compute AutoPilot result ---
        Optional<Rotation2d> fieldEntryAngle = m_entryAngle
                .map(a -> a.plus(m_target.getReference().getRotation()));

        APResult out;
        if (fieldEntryAngle.isPresent()) {
            out = kAutopilot.calculate(pose, fieldSpeeds,
                    m_target.withEntryAngle(fieldEntryAngle.get()));
        } else {
            out = kAutopilot.calculate(pose, fieldSpeeds,
                    m_target.withoutEntryAngle());
        }

        // --- 4. Apply single control request ---
        // FieldCentricFacingAngle handles field→module kinematics and heading PID
        // internally. No frame conversion, no code-path branching.
        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetDirection(out.targetAngle()));

        // --- 5. Diagnostic logging ---
        logAPOutputs(out, pose, robotSpeeds, fieldSpeeds);

        if (m_cycleCount % MODULE_LOG_INTERVAL == 0) {
            logModuleDiagnostics();
        }
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        m_drivetrain.setControl(m_request
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(m_drivetrain.getState().Pose.getRotation()));

        Pose2d finalPose = m_drivetrain.getState().Pose;
        double finalDist = finalPose.getTranslation()
                .getDistance(m_target.getReference().getTranslation());
        double finalHdgErr = m_target.getReference().getRotation()
                .minus(finalPose.getRotation()).getDegrees();

        fLogger.log(
                "END %s | interrupted=%s dist=%.3fm hdgErr=%.1f° final(%.2f,%.2f,%.1f°) target(%.2f,%.2f,%.1f°)",
                getName(), interrupted,
                finalDist, finalHdgErr,
                finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees(),
                m_target.getReference().getX(), m_target.getReference().getY(),
                m_target.getReference().getRotation().getDegrees());
    }

    @Override
    public String getName() {
        return "APv2(" + commandName + ")";
    }

    // -----------------------------------------------------------------------
    // Logging
    // -----------------------------------------------------------------------

    /**
     * Logs AutoPilot inputs/outputs every cycle.
     *
     * <p>
     * <b>How to read these in AdvantageScope / SmartDashboard:</b>
     * <ul>
     * <li>{@code APv2/AP_vx_mps}, {@code AP_vy_mps} — field-relative velocity
     * AutoPilot is commanding. If THESE oscillate, the problem is upstream
     * of driveGains (AutoPilot itself or the speed input).</li>
     * <li>{@code APv2/fieldSpeed_*} vs {@code robotSpeed_*} — compare the
     * speed conversion. If {@code fieldSpeed} looks wrong when the robot
     * is rotated, the conversion is broken.</li>
     * <li>{@code APv2/headingError_deg} — heading error fed to
     * FieldCentricFacingAngle's internal PID. Large/oscillating values
     * inject rotational velocity into module drive motors.</li>
     * </ul>
     */
    private void logAPOutputs(APResult out, Pose2d pose,
            ChassisSpeeds robotSpeeds, ChassisSpeeds fieldSpeeds) {
        double apVx = out.vx().baseUnitMagnitude();
        double apVy = out.vy().baseUnitMagnitude();
        double apSpeed = Math.hypot(apVx, apVy);
        double headingErr = out.targetAngle().minus(pose.getRotation()).getDegrees();
        double dist = pose.getTranslation().getDistance(m_target.getReference().getTranslation());

        // AP command outputs
        SmartDashboard.putNumber(NT + "AP_vx_mps", apVx);
        SmartDashboard.putNumber(NT + "AP_vy_mps", apVy);
        SmartDashboard.putNumber(NT + "AP_speed_mps", apSpeed);
        SmartDashboard.putNumber(NT + "AP_targetAngle_deg", out.targetAngle().getDegrees());

        // State
        SmartDashboard.putNumber(NT + "headingError_deg", headingErr);
        SmartDashboard.putNumber(NT + "distToTarget_m", dist);

        // Speed comparison: robot-relative vs field-relative
        SmartDashboard.putNumber(NT + "robotSpeed_vx", robotSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber(NT + "robotSpeed_vy", robotSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber(NT + "robotSpeed_omega", robotSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber(NT + "fieldSpeed_vx", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber(NT + "fieldSpeed_vy", fieldSpeeds.vyMetersPerSecond);

        fLogger.log("vx:%.3f vy:%.3f spd:%.3f hdg:%.1f° dist:%.3fm | rVx:%.3f rVy:%.3f fVx:%.3f fVy:%.3f",
                apVx, apVy, apSpeed, headingErr, dist,
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond,
                fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    }

    /**
     * Logs per-module drive motor closed-loop data (throttled).
     *
     * <p>
     * <b>How to interpret:</b>
     * <ul>
     * <li>{@code clRef} (closedLoopReference) — velocity the motor is TOLD to
     * achieve. If this oscillates, the swerve kinematics or heading PID is
     * the source, not driveGains.</li>
     * <li>{@code clErr} (closedLoopError) — gap between reference and actual.
     * Sign-flipping every few cycles = oscillation.</li>
     * <li>{@code clOut} (closedLoopOutput) — PID output voltage. If this
     * alternates ±, the motor is hunting. Reduce kP or increase kD.</li>
     * <li>{@code vel} — actual motor velocity (rot/s). Compare to
     * {@code clRef}.</li>
     * <li>{@code volts} — actual voltage applied to the motor.</li>
     * </ul>
     */
    private void logModuleDiagnostics() {
        @SuppressWarnings("unchecked")
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = m_drivetrain.getModules();

        for (int i = 0; i < modules.length; i++) {
            TalonFX driveMotor = modules[i].getDriveMotor();
            String p = NT + "M" + i + "/";

            SmartDashboard.putNumber(p + "clRef", driveMotor.getClosedLoopReference().getValueAsDouble());
            SmartDashboard.putNumber(p + "clErr", driveMotor.getClosedLoopError().getValueAsDouble());
            SmartDashboard.putNumber(p + "clOut", driveMotor.getClosedLoopOutput().getValueAsDouble());
            SmartDashboard.putNumber(p + "vel", driveMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(p + "volts", driveMotor.getMotorVoltage().getValueAsDouble());
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /**
     * Converts robot-relative ChassisSpeeds to field-relative by rotating the
     * translational component by the robot's heading.
     */
    private static ChassisSpeeds toFieldRelative(ChassisSpeeds robotSpeeds, Rotation2d robotAngle) {
        Translation2d vel = new Translation2d(
                robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond);
        Translation2d fieldVel = vel.rotateBy(robotAngle);
        return new ChassisSpeeds(
                fieldVel.getX(),
                fieldVel.getY(),
                robotSpeeds.omegaRadiansPerSecond);
    }
}
