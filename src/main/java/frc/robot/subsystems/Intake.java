// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakePreferences;
import frc.robot.lib.FinneyLogger;
import frc.robot.sim.SimProfiles;

// todo add a button on elastic to zero the intake
public class Intake extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  private final TalonFX intakeMotor = new TalonFX(Constants.CANBus.INTAKE_MOTOR);
  private final TalonFX pickupMotor = new TalonFX(Constants.CANBus.PICKUP_MOTOR);

  private final MotionMagicVoltage intakePositionRequest = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage pickupVelocityRequest = new MotionMagicVelocityVoltage(0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private int settleCount = 0;
  private int stallCount = 0;
  private double intakePositionTarget = 0;
  private boolean isIntakeDeployed = false;
  private boolean isPickupActive = false;

  public Intake() {
    configureIntakeMotor();
    configurePickupMotor();

    // Sim profiles handle sim-specific physics; they early-return on real robot
    SimProfiles.initIntake(intakeMotor);
    SimProfiles.initPickup(pickupMotor);
  }

  // ── Motor Configuration ──────────────────────────────────────────────────

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = IntakePreferences.DEPLOY_KP;
    config.Slot0.kI = IntakePreferences.DEPLOY_KI;
    config.Slot0.kD = IntakePreferences.DEPLOY_KD;
    config.Slot0.kS = IntakePreferences.DEPLOY_KS;
    config.Slot0.kV = IntakePreferences.DEPLOY_KV;
    config.Slot0.kG = IntakePreferences.DEPLOY_KG;

    config.MotionMagic.MotionMagicCruiseVelocity = IntakePreferences.DEPLOY_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = IntakePreferences.DEPLOY_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = IntakePreferences.DEPLOY_JERK;

    config.Voltage.PeakForwardVoltage = IntakePreferences.PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = IntakePreferences.PEAK_REVERSE_VOLTAGE;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakePreferences.DEPLOY_STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePreferences.DEPLOY_SUPPLY_LIMIT;

    // Soft limits prevent driving beyond mechanical range
    double softForward = Math.max(IntakePreferences.INTAKE_MOTOR_OUT, IntakePreferences.INTAKE_MOTOR_IN)
        + IntakePreferences.SOFT_LIMIT_MARGIN;
    double softReverse = Math.min(IntakePreferences.INTAKE_MOTOR_OUT, IntakePreferences.INTAKE_MOTOR_IN)
        - IntakePreferences.SOFT_LIMIT_MARGIN;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softForward;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softReverse;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeMotor.getConfigurator().apply(config);
    fLogger.log("Intake motor configured (deploy)");

    SmartDashboard.putBoolean("Intake/ResetDeployEncoder", false);
  }

  private void configurePickupMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = IntakePreferences.PICKUP_KP;
    config.Slot0.kI = IntakePreferences.PICKUP_KI;
    config.Slot0.kD = IntakePreferences.PICKUP_KD;
    config.Slot0.kS = IntakePreferences.PICKUP_KS;
    config.Slot0.kV = IntakePreferences.PICKUP_KV;

    config.MotionMagic.MotionMagicCruiseVelocity = 10;
    config.MotionMagic.MotionMagicAcceleration = 50;
    config.MotionMagic.MotionMagicJerk = 0;

    config.Voltage.PeakForwardVoltage = IntakePreferences.PEAK_FORWARD_VOLTAGE;
    config.Voltage.PeakReverseVoltage = IntakePreferences.PEAK_REVERSE_VOLTAGE;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakePreferences.PICKUP_STATOR_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakePreferences.PICKUP_SUPPLY_LIMIT;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    pickupMotor.getConfigurator().apply(config);
    fLogger.log("Pickup motor configured (roller)");
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    double currentPosition = intakeMotor.getPosition().getValueAsDouble();
    double positionError = Math.abs(currentPosition - intakePositionTarget);

    // Settle counting: require N consecutive cycles within tolerance
    if (positionError < IntakePreferences.POSITION_TOLERANCE) {
      settleCount++;
    } else {
      settleCount = 0;
    }

    // Stall detection for the deploy motor
    double statorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
    double velocity = intakeMotor.getVelocity().getValueAsDouble();
    boolean motorCommanded = Math.abs(intakeMotor.getClosedLoopReference().getValueAsDouble()
        - currentPosition) > IntakePreferences.POSITION_TOLERANCE;

    if (motorCommanded && Math.abs(velocity) < 0.5 && statorCurrent > IntakePreferences.STALL_CURRENT_THRESHOLD) {
      stallCount++;
      if (stallCount >= IntakePreferences.STALL_CYCLES_THRESHOLD) {
        fLogger.log("STALL DETECTED — stopping intake deploy motor (%.1fA)", statorCurrent);
        intakeMotor.setControl(neutralRequest);
        // Accept wherever we are as the new target to prevent re-stalling
        intakePositionTarget = currentPosition;
        stallCount = 0;
      }
    } else {
      stallCount = 0;
    }

    checkForResetEncoder();
    publishTelemetry();
  }

  // ── State Queries ────────────────────────────────────────────────────────

  private boolean isAtTarget() {
    return settleCount >= IntakePreferences.SETTLE_COUNT;
  }

  public boolean isIntakeExtended() {
    return isIntakeDeployed;
  }

  public boolean isPickupRunning() {
    return isPickupActive;
  }

  // ── Low-Level Motor Actions ──────────────────────────────────────────────

  private void setIntakePosition(double position) {
    intakeMotor.setControl(intakePositionRequest.withPosition(position));
    intakePositionTarget = position;
    settleCount = 0;
    stallCount = 0;
  }

  private void setPickupVelocity(double velocity) {
    pickupMotor.setControl(pickupVelocityRequest.withVelocity(velocity));
    isPickupActive = true;
  }

  private void stopPickupMotor() {
    pickupMotor.setControl(neutralRequest);
    isPickupActive = false;
  }

  // ── Intake Deploy Positions ──────────────────────────────────────────────

  private void deployOut() {
    isIntakeDeployed = true;
    setIntakePosition(IntakePreferences.INTAKE_MOTOR_OUT);
    fLogger.log("Intake → OUT (%.1f)", IntakePreferences.INTAKE_MOTOR_OUT);
  }

  private void deployIn() {
    isIntakeDeployed = false;
    setIntakePosition(IntakePreferences.INTAKE_MOTOR_IN);
    fLogger.log("Intake → IN (%.1f)", IntakePreferences.INTAKE_MOTOR_IN);
  }

  private void deployCenter() {
    isIntakeDeployed = true;
    setIntakePosition(IntakePreferences.INTAKE_MOTOR_CENTER);
    fLogger.log("Intake CENTER (%.1f)", IntakePreferences.INTAKE_MOTOR_CENTER);
  }

  // ── Pickup Roller Speeds ─────────────────────────────────────────────────

  private void pickupRollIn() {
    setPickupVelocity(IntakePreferences.PICKUP_MOTOR_IN);
    fLogger.log("Pickup IN (%.1f rps)", IntakePreferences.PICKUP_MOTOR_IN);
  }

  private void pickupRollOut() {
    setPickupVelocity(IntakePreferences.PICKUP_MOTOR_OUT);
    fLogger.log("Pickup OUT (%.1f rps)", IntakePreferences.PICKUP_MOTOR_OUT);
  }

  // ── Public Commands ──────────────────────────────────────────────────────

  /**
   * Deploy the intake to the OUT position and hold there.
   * The motor continues holding position after the command finishes.
   */
  public Command runIntakeOut() {
    return Commands.sequence(
        runOnce(() -> deployOut()),
        Commands.waitUntil(this::isAtTarget))
        .withName("Run Intake Out");
  }

  /**
   * Retract the intake to the IN position and hold there.
   * The motor continues holding position after the command finishes.
   */
  public Command runIntakeIn() {
    return Commands.sequence(
        runOnce(() -> deployIn()),
        Commands.waitUntil(this::isAtTarget))
        .withName("Run Intake In");
  }

  /**
   * Move the intake to the CENTER position and hold there.
   */
  public Command runIntakeCenter() {
    return Commands.sequence(
        runOnce(() -> deployCenter()),
        Commands.waitUntil(this::isAtTarget))
        .withName("Run Intake Center");
  }

  /** Run the pickup rollers inward (intaking game pieces). */
  public Command runPickupIn() {
    return startEnd(() -> pickupRollIn(), () -> stopPickupMotor())
        .withName("Run Pickup In");
  }

  /** Run the pickup rollers outward (ejecting game pieces). */
  public Command runPickupOut() {
    return runOnce(() -> pickupRollOut())
        .withName("Run Pickup Out");
  }

  /** Stop the pickup rollers. */
  public Command runPickupStop() {
    return runOnce(() -> stopPickupMotor())
        .withName("Run Pickup Stop");
  }

  // Named/SmartDashboard-publishing overloads
  public Command runPickupOut(String name) {
    Command cmd = runPickupOut().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  public Command runPickupIn(String name) {
    Command cmd = runPickupIn().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  public Command runPickupStop(String name) {
    Command cmd = runPickupStop().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /** Stop pickup then retract intake. */
  public Command runRetractIntake() {
    return Commands.sequence(
        runPickupStop(),
        runIntakeIn())
        .withName("Retract Intake");
  }

  public void checkForResetEncoder() {
    boolean value = SmartDashboard.getBoolean("Intake/ResetDeployEncoder", false);
    if (value) {
      intakeMotor.setPosition(0);
      SmartDashboard.putBoolean("Intake/ResetDeployEncoder", false);
    }
  }

  // ── Telemetry ────────────────────────────────────────────────────────────

  private void publishTelemetry() {
    double position = intakeMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Intake/DeployPosition", position);
    SmartDashboard.putNumber("Intake/DeployTarget", intakePositionTarget);
    SmartDashboard.putNumber("Intake/DeployError", Math.abs(position - intakePositionTarget));
    SmartDashboard.putNumber("Intake/DeployVelocity", intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/DeployCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/PickupCurrent", pickupMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/PickupVelocity", pickupMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/IsDeployed", isIntakeDeployed);
    SmartDashboard.putBoolean("Intake/IsPickupActive", isPickupActive);
    SmartDashboard.putBoolean("Intake/AtTarget", isAtTarget());
    SmartDashboard.putNumber("Intake/SettleCount", settleCount);
    SmartDashboard.putNumber("Intake/StallCount", stallCount);
  }

  public void publishMotorCurrents() {
    SmartDashboard.putNumber("Intake/IntakeCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/PickupCurrent", pickupMotor.getStatorCurrent().getValueAsDouble());
  }
}
