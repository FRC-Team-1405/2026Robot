package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPhysicalProperties;
import frc.robot.Constants.ShooterPIDConfig;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.constants.FeatureSwitches;
import frc.robot.lib.FinneyLogger;
import frc.robot.lib.MotorSim.MotorSim_Mech;

public class Shooter extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName(),
      FeatureSwitches.ENABLE_SUBSYSTEM_LOGGING);

  private final TalonFX shooterMotor1 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_2);
  private final TalonFX shooterMotor3 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_3);

  private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final NeutralOut brake = new NeutralOut();

  private final MotorSim_Mech shooterMotorSimMech = new MotorSim_Mech("Shooter/Mech2d");

  private final Alert configAlert = new Alert("Shooter motor configuration failed", AlertType.kError);

  private LinearFilter filter = LinearFilter.movingAverage(ShooterPIDConfig.FILTER_WINDOW);

  // Rolling std dev via E[x^2] - E[x]^2
  private LinearFilter velocityMeanFilter = LinearFilter.movingAverage(ShooterPIDConfig.FILTER_WINDOW);
  private LinearFilter velocityMeanSqFilter = LinearFilter.movingAverage(ShooterPIDConfig.FILTER_WINDOW);

  private double highError = 0.0;
  private double lowError = 0.0;

  private int settleCount = 0;
  private int shotCount = 0;

  private double shooterTarget = 0.0;
  private double shooterStartTimestamp = 0.0;
  private double timeToLockSeconds = 0.0;

  private boolean locked = false;
  private boolean wasLocked = false;

  private Supplier<AngularVelocity> requestedSpeed = () -> Constants.ShooterPreferences.LONG;

  public boolean isReadyToFire() {
    return locked;
  }

  private void setupMotors() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = ShooterPIDConfig.KP;
    cfg.Slot0.kI = ShooterPIDConfig.KI;
    cfg.Slot0.kD = ShooterPIDConfig.KD;
    cfg.Slot0.kV = ShooterPIDConfig.KV;
    cfg.Slot0.kS = ShooterPIDConfig.KS;

    cfg.Voltage.withPeakForwardVoltage(Volts.of(ShooterPIDConfig.PEAK_FORWARD_VOLTAGE))
        .withPeakReverseVoltage(Volts.of(ShooterPIDConfig.PEAK_REVERSE_VOLTAGE));

    cfg.MotionMagic.MotionMagicAcceleration = ShooterPIDConfig.MOTION_MAGIC_ACCELERATION;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterMotor1.getConfigurator().apply(cfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure shooter motor. Error: " + status.toString());
      configAlert.set(true);
      shooterMotor1.setControl(brake);
    }
  }

  /** Creates a new Shooter. */
  public Shooter() {
    setupMotors();
    simulationInit();
    shooterMotor2.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    shooterMotor3.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    SmartDashboard.putNumber("Shooter/TestTargetRPS", 10.0);
    shooterStop();
  }

  private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    AngularVelocity target = speed.get();
    shooterMotor1.setControl(velocityVoltage.withVelocity(target));
    shooterTarget = target.in(RotationsPerSecond);
    shooterStartTimestamp = Timer.getFPGATimestamp();
    timeToLockSeconds = 0.0;
    fLogger.log("setShooterSpeed called: target=%.2f RPS", shooterTarget);
  }

  private void shooterStop() {
    shooterTarget = 0.0;
    shooterMotor1.setControl(brake);
    fLogger.log("shooterStop called");
  }

  private void setRequestedSpeed(Supplier<AngularVelocity> speed) {
    requestedSpeed = speed;
    setShooterSpeed(requestedSpeed);
  }

  /**
   * get the desired robot distance to the center of the hub for the set shooter
   * speed.
   * 
   * @return
   */
  public Supplier<Double> getDistanceFromSpeed() {
    return ShooterPreferences.SHOOTER_SPEED_TO_DISTANCE.get(requestedSpeed.get());
  }

  public void increaseDistanceForSpeed() {
    Double value = ShooterPreferences.SHOOTER_SPEED_TO_DISTANCE.get(requestedSpeed.get()).get();
    ShooterPreferences.SHOOTER_SPEED_TO_DISTANCE.put(requestedSpeed.get(), () -> value + 0.1);
  }

  public void descreaseDistanceForSpeed() {
    Double value = ShooterPreferences.SHOOTER_SPEED_TO_DISTANCE.get(requestedSpeed.get()).get();
    ShooterPreferences.SHOOTER_SPEED_TO_DISTANCE.put(requestedSpeed.get(), () -> value - 0.1);
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setShooterSpeed(speed), this);
  }

  public Command runShooterAuto(Supplier<AngularVelocity> requestedSpeed) {
    return Commands.startEnd(
        () -> setShooterSpeed(requestedSpeed),
        () -> stopShooter(),
        this);
  }

  public Command runShooter() {
    return Commands.runOnce(() -> setShooterSpeed(requestedSpeed), this);
  }

  /**
   * Reads Shooter/TestTargetRPS from SmartDashboard and spins up to that speed.
   */
  public Command runShooterAtTestRPS() {
    return Commands.runOnce(
        () -> setShooterSpeed(
            () -> RotationsPerSecond.of(SmartDashboard.getNumber("Shooter/TestTargetRPS", 10.0))),
        this);
  }

  public Command runSetRequestedSpeed(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setRequestedSpeed(speed));
  }

  public Command stopShooter() {
    return Commands.runOnce(() -> shooterStop(), this);
  }

  @Override
  public void periodic() {
    // --- Velocities (cache to avoid redundant CAN calls) ---
    double motor1RPS = shooterMotor1.getVelocity().getValueAsDouble();
    double motor2RPS = shooterMotor2.getVelocity().getValueAsDouble();
    double motor3RPS = shooterMotor3.getVelocity().getValueAsDouble();

    // --- Current draw ---
    double motor1SupplyCurrent = shooterMotor1.getSupplyCurrent().getValueAsDouble();
    double motor2SupplyCurrent = shooterMotor2.getSupplyCurrent().getValueAsDouble();
    double motor3SupplyCurrent = shooterMotor3.getSupplyCurrent().getValueAsDouble();
    double motor1TorqueCurrent = shooterMotor1.getTorqueCurrent().getValueAsDouble();
    double motor2TorqueCurrent = shooterMotor2.getTorqueCurrent().getValueAsDouble();
    double motor3TorqueCurrent = shooterMotor3.getTorqueCurrent().getValueAsDouble();

    double cumulativeMotorStatorCurrent = shooterMotor1.getStatorCurrent().getValueAsDouble()
        + shooterMotor2.getStatorCurrent().getValueAsDouble() + shooterMotor3.getStatorCurrent().getValueAsDouble();

    // --- Output & supply voltage ---
    double motor1OutputVoltage = shooterMotor1.getMotorVoltage().getValueAsDouble();
    double motor2OutputVoltage = shooterMotor2.getMotorVoltage().getValueAsDouble();
    double motor3OutputVoltage = shooterMotor3.getMotorVoltage().getValueAsDouble();
    double supplyVoltage = shooterMotor1.getSupplyVoltage().getValueAsDouble();

    // --- Temperatures ---
    double motor1Temp = shooterMotor1.getDeviceTemp().getValueAsDouble();
    double motor2Temp = shooterMotor2.getDeviceTemp().getValueAsDouble();
    double motor3Temp = shooterMotor3.getDeviceTemp().getValueAsDouble();

    // --- Closed loop diagnostics ---
    double averageError = filter.calculate(shooterMotor1.getClosedLoopError().getValueAsDouble());
    double error = shooterMotor1.getClosedLoopError().getValueAsDouble();
    double target = shooterMotor1.getClosedLoopReference().getValueAsDouble();
    double range = locked ? ShooterPreferences.WIDE : ShooterPreferences.TIGHT;

    // --- Rolling std dev: sqrt(E[x^2] - E[x]^2) ---
    double mean = velocityMeanFilter.calculate(motor1RPS);
    double meanSq = velocityMeanSqFilter.calculate(motor1RPS * motor1RPS);
    double stdDev = Math.sqrt(Math.max(0.0, meanSq - mean * mean));

    // --- Ball exit velocity estimation ---
    // wheel RPS = motor RPS * gear ratio; exit vel = wheel RPS * wheel
    // circumference
    double exitVelocityFPS = motor1RPS
        * ShooterPhysicalProperties.MOTOR_TO_WHEEL_GEAR_RATIO
        * Math.PI * (ShooterPhysicalProperties.FLYWHEEL_DIAMETER_INCHES / 12.0);

    // --- Follower sync check ---
    double motor2RPSDelta = motor1RPS - motor2RPS;
    double motor3RPSDelta = motor1RPS - motor3RPS;
    double differentialCurrentDraw = Math.abs(motor1SupplyCurrent - motor2SupplyCurrent);

    // --- High/low error envelope ---
    if (error >= highError) {
      highError = error;
    } else {
      highError -= 1;
    }

    if (error <= lowError) {
      lowError = error;
    } else {
      lowError += 1;
    }

    // --- Settle / lock logic ---
    if (MathUtil.isNear(shooterTarget, target, ShooterPIDConfig.TARGET_MATCH_TOLERANCE) && Math.abs(error) < range) {
      if (settleCount < ShooterPreferences.STABLE_COUNT) {
        settleCount += 1;
      }
    } else {
      settleCount = 0;
    }

    locked = settleCount >= ShooterPreferences.STABLE_COUNT && shooterTarget > 0.0;

    // --- Time to lock ---
    if (locked && !wasLocked) {
      timeToLockSeconds = Timer.getFPGATimestamp() - shooterStartTimestamp;
      fLogger.log("Shooter locked: timeToLock=%.2fs, target=%.2f RPS", timeToLockSeconds, shooterTarget);
    }

    // --- Shot counter: locked -> unlocked while target is still set ---
    if (!locked && wasLocked && shooterTarget > 0.0) {
      shotCount++;
      fLogger.log("Shot detected: count=%d", shotCount);
    }

    wasLocked = locked;

    // --- Mechanism2d update ---
    shooterMotorSimMech.update(shooterMotor1.getPosition(), shooterMotor1.getVelocity());

    // --- SmartDashboard ---
    if (FeatureSwitches.ENABLE_SUBSYSTEM_LOGGING) {
      // Velocity
      SmartDashboard.putNumber("Shooter/Motor1RPS", motor1RPS);
      SmartDashboard.putNumber("Shooter/Motor2RPS", motor2RPS);
      SmartDashboard.putNumber("Shooter/Motor3RPS", motor3RPS);
      SmartDashboard.putNumber("Shooter/TargetRPS", shooterTarget);
      // SmartDashboard.putNumber("Shooter/Motor1StdDev", stdDev);
      // SmartDashboard.putNumber("Shooter/BallExitVelocityFPS", exitVelocityFPS);

      // Follower sync
      SmartDashboard.putNumber("Shooter/Motor2RPSDelta", motor2RPSDelta);
      SmartDashboard.putNumber("Shooter/Motor3RPSDelta", motor3RPSDelta);

      // Supply & output voltage
      // SmartDashboard.putNumber("Shooter/SupplyVoltage", supplyVoltage);
      // SmartDashboard.putNumber("Shooter/Motor1OutputVoltage", motor1OutputVoltage);
      // SmartDashboard.putNumber("Shooter/Motor2OutputVoltage", motor2OutputVoltage);
      // SmartDashboard.putNumber("Shooter/Motor3OutputVoltage", motor3OutputVoltage);

      // Current
      // SmartDashboard.putNumber("Shooter/Motor1SupplyCurrent", motor1SupplyCurrent);
      // SmartDashboard.putNumber("Shooter/Motor2SupplyCurrent", motor2SupplyCurrent);
      // SmartDashboard.putNumber("Shooter/Motor3SupplyCurrent", motor3SupplyCurrent);
      // SmartDashboard.putNumber("Shooter/DifferentialCurrent",
      // differentialCurrentDraw);
      SmartDashboard.putNumber("Shooter/Motor1TorqueCurrent", motor1TorqueCurrent);
      SmartDashboard.putNumber("Shooter/Motor2TorqueCurrent", motor2TorqueCurrent);
      SmartDashboard.putNumber("Shooter/Motor3TorqueCurrent", motor3TorqueCurrent);
      SmartDashboard.putNumber("Shooter/CumulativeStatorCurrent", cumulativeMotorStatorCurrent);

      // Temperature
      // SmartDashboard.putNumber("Shooter/Motor1Temp", motor1Temp);
      // SmartDashboard.putNumber("Shooter/Motor2Temp", motor2Temp);
      // SmartDashboard.putNumber("Shooter/Motor3Temp", motor3Temp);

      // PID diagnostics
      SmartDashboard.putNumber("Shooter/Error", error);
      SmartDashboard.putNumber("Shooter/AverageError", averageError);
      // SmartDashboard.putNumber("Shooter/HighError", highError);
      // SmartDashboard.putNumber("Shooter/LowError", lowError);
      SmartDashboard.putNumber("Shooter/SettleCount", settleCount);

      // Lock & shot
      SmartDashboard.putBoolean("Shooter/Locked", locked);
      SmartDashboard.putNumber("Shooter/TimeToLockSeconds", timeToLockSeconds);
      SmartDashboard.putNumber("Shooter/ShotCount", shotCount);

      // Speed preset indicators
      SmartDashboard.putBoolean("Shooter/Long Speed", requestedSpeed.get() == Constants.ShooterPreferences.LONG);
      SmartDashboard.putBoolean("Shooter/Medium Speed", requestedSpeed.get() == Constants.ShooterPreferences.MEDIUM);
      SmartDashboard.putBoolean("Shooter/Short Speed", requestedSpeed.get() == Constants.ShooterPreferences.SHORT);

      // Shooter Distance
      SmartDashboard.putNumber("Shooter/Requested Speed", requestedSpeed.get().in(RotationsPerSecond));
      SmartDashboard.putNumber("Shooter/Desired Distance", getDistanceFromSpeed().get());
    }
  }

  public void simulationInit() {
    // Use physical properties from Constants to create a more realistic sim
    // profile.
    // The moment of inertia in Constants is for the flywheel (wheel side). Reflect
    // it to the motor/rotor side: I_rotor = I_wheel / (gearRatio^2)
    final double gearRatio = Constants.ShooterPhysicalProperties.MOTOR_TO_WHEEL_GEAR_RATIO;
    final double flywheelInertia = Constants.ShooterPhysicalProperties.FLYWHEEL_MOMENT_OF_INERTIA;
    final double rotorInertia = flywheelInertia / (gearRatio * gearRatio);

    // Convert the (approximate) projectile mass (lbs) to kg for gravity load
    final double loadMassKg = Pounds.of(Constants.ShooterPhysicalProperties.FUEL_WEIGHT_LBS).in(Kilograms);

    // Use flywheel radius as the arm for gravity torque (meters)
    final double armMeters = Inches.of(Constants.ShooterPhysicalProperties.FLYWHEEL_DIAMETER_INCHES / 2.0)
        .in(Meters);

    // Small viscous friction estimate (N*m per rad/s). Tweak if needed.
    final double viscousCoeff = 0.01;

    // There are three motors on the shooter (one leader + two followers)
    final int numMotors = 3;

    frc.robot.sim.sjc.PhysicsSim_SJC.getInstance().addTalonFX(shooterMotor1, rotorInertia, loadMassKg,
        armMeters, viscousCoeff, numMotors, gearRatio);
  }

  @Override
  public void simulationPeriodic() {
    frc.robot.sim.sjc.PhysicsSim_SJC.getInstance().run();
  }
}