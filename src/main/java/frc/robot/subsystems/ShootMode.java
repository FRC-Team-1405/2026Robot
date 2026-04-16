package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.ShootMode.Mode;

public class ShootMode {
    private SwerveFeatures swerveFeatures;
    private CommandSwerveDrivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private CommandXboxController joystick;

    public ShootMode(CommandSwerveDrivetrain drivetrain, SwerveFeatures swerveFeatures, Intake intake, Indexer indexer,
            Shooter shooter,
            CommandXboxController joystick) {
        this.shooter = shooter;
        this.joystick = joystick;
        this.intake = intake;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.swerveFeatures = swerveFeatures;
    }

    public enum Mode {
        SHORT,
        MEDIUM,
        LONG,
        DYNAMIC
    }

    private Mode activeMode = Mode.MEDIUM;

    public void setMode(Mode mode) {
        if (mode.equals(Mode.SHORT)) {
            shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT);
        } else if (mode.equals(Mode.MEDIUM)) {
            shooter.runSetRequestedSpeed(() -> ShooterPreferences.MEDIUM);
        } else if (mode.equals(Mode.LONG)) {
            shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG);
        } else if (mode.equals(Mode.DYNAMIC)) {
            CommandScheduler.getInstance().schedule(AutoFire.DynamicTeleop(shooter, indexer,
                    () -> ShooterPreferences.INDEXER_VELOCITY, intake, () -> swerveFeatures
                            .getDistanceToHub(drivetrain,
                                    FieldConstants.ALLIANCE_HUB_POSITION),
                    joystick.rightBumper()));
        } else {
            shooter.stopShooter();
        }
    }

    public Trigger isNonDynamicShootMode() {
        return new Trigger(() -> !activeMode.equals(Mode.DYNAMIC));
    }
}
