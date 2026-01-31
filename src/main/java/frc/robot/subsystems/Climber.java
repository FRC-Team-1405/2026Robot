// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.FinneyLogger;
import frc.robot.sim.SimProfiles;

public class Climber extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());
  /** Creates a new Climber. */
  private TalonFX motor = new TalonFX(45);
  private final PositionVoltage motorPosition = new PositionVoltage(0);
  private final NeutralOut stop = new NeutralOut();
  private TalonFX grabber = new TalonFX(37);
  private final PositionVoltage grabberPosition = new PositionVoltage(0);

  private int climberSettleCount = 0;
  private int grabberSettleCount = 0;

  private final double POSITION_TOLERANCE = 1.0;
  private final int SETTLE_MAX = 3;

  private static final double CLIMBER_CURRENT_LIMIT = 40.0;

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Climber Voltage", motor.getMotorVoltage().getValueAsDouble());
    fLogger.log("Motor voltage = " + motor.getMotorVoltage().getValueAsDouble() + " Motor position = "
        + motor.getPosition().getValueAsDouble());

    if (Math.abs(motor.getClosedLoopError().getValueAsDouble()) < POSITION_TOLERANCE) {
      climberSettleCount += 1;
    } else {
      climberSettleCount = 0;
    }

    if (Math.abs(grabber.getClosedLoopError().getValueAsDouble()) < POSITION_TOLERANCE) {
      grabberSettleCount += 1;
    } else {
      grabberSettleCount = 0;
    }
  }

  private boolean isClimberAtTarget() {
    return (climberSettleCount >= SETTLE_MAX);
  }

  private boolean isGrabberAtTarget() {
    return (grabberSettleCount >= SETTLE_MAX);
  }

  public void setupMotors() {
    TalonFXConfiguration Climber_cfg = new TalonFXConfiguration();
    TalonFXConfiguration Grabber_cfg = new TalonFXConfiguration();

    MotionMagicConfigs Climber_mm = Climber_cfg.MotionMagic;
    // Climber_mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))
    // .withMotionMagicAcceleration(RotationsPerSecond.of(15));

    StatusCode Climber_status_slave = StatusCode.StatusCodeNotInitialized;
    StatusCode Grabber_status_slave = StatusCode.StatusCodeNotInitialized;

    // SoftwareLimitSwitchConfigs soft = Climber_cfg.SoftwareLimitSwitch;
    // soft.ForwardSoftLimitEnable = true;
    // soft.ForwardSoftLimitThreshold = 5.1; // mechanism rotations (after gear
    // ratio)
    // soft.ReverseSoftLimitEnable = true;
    // soft.ReverseSoftLimitThreshold = 0.0;

    // SoftwareLimitSwitchConfigs softGrabber = Grabber_cfg.SoftwareLimitSwitch;
    // softGrabber.ForwardSoftLimitEnable = true;
    // softGrabber.ForwardSoftLimitThreshold = 3.0;
    // softGrabber.ReverseSoftLimitEnable = true;
    // softGrabber.ReverseSoftLimitThreshold = 0.0;

    Slot0Configs Climber_slot0 = Climber_cfg.Slot0;
    Climber_slot0.kS = 0;
    Climber_slot0.kV = 0.0;
    Climber_slot0.kA = 0.0;
    Climber_slot0.kP = 3; // proportional
    Climber_slot0.kI = 0;
    Climber_slot0.kD = 0.0;
    Climber_slot0.kG = 0;

    Slot0Configs Grabber_slot0 = Grabber_cfg.Slot0;
    Grabber_slot0.kS = 0;
    Grabber_slot0.kV = 0.0;
    Grabber_slot0.kA = 0.0;
    Grabber_slot0.kP = 3;
    Grabber_slot0.kI = 0;
    Grabber_slot0.kD = 0.0;
    Grabber_slot0.kG = 0;

    // CurrentLimitsConfigs limits = Climber_cfg.CurrentLimits;
    // limits.SupplyCurrentLimitEnable = true;
    // limits.SupplyCurrentLimit = 30;
    // limits.StatorCurrentLimitEnable = true;
    // limits.StatorCurrentLimit = 40;

    CurrentLimitsConfigs grabberLimits = Grabber_cfg.CurrentLimits;
    grabberLimits.SupplyCurrentLimitEnable = true;
    grabberLimits.SupplyCurrentLimit = 30;
    grabberLimits.StatorCurrentLimitEnable = true;
    grabberLimits.StatorCurrentLimit = 40;
    grabberLimits.SupplyCurrentLimit = 30;
    grabberLimits.StatorCurrentLimitEnable = true;
    grabberLimits.StatorCurrentLimit = 40;

    for (int i = 0; i < 5; ++i) {
      Climber_status_slave = motor.getConfigurator().apply(Climber_cfg);
      if (Climber_status_slave.isOK())
        break;
    }
    if (!Climber_status_slave.isOK()) {
      fLogger.log("Could not configure Climber slaveMotor. Error: " + Climber_status_slave.toString());
    }

    for (int i = 0; i < 5; ++i) {
      Grabber_status_slave = grabber.getConfigurator().apply(Grabber_cfg);
      if (Grabber_status_slave.isOK())
        break;
    }
    if (!Grabber_status_slave.isOK()) {
      fLogger.log("Could not configure Grabber slaveMotor. Error: " + Grabber_status_slave.toString());
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  private double MAX_DISTANCE = 100.0;
  private double MIN_DISTANCE = 0.0;
  private double MAX_GRABBER_DISTANCE = 50.0;
  private double MIN_GRABBER_DISTANCE = 0.0;

  public void move(double position) {
    // position = MAX_DISTANCE * (position);
    // motor.setControl(new MotionMagicVoltage(position));
    motor.setControl(motorPosition.withPosition(position));
  }

  public void moveGrabber(double position) {
    // motor.setControl(new MotionMagicVoltage(position));
    grabber.set(0.2);
  }

  public Climber() {
    setupMotors();
    SimProfiles.initClimber(motor);
    SimProfiles.initGrabber(grabber);
  }

  // function to climb up - motor forward direction
  private void climbUp() {
    move(MAX_DISTANCE);
    fLogger.log("Climb up ");
    // motor.set(0.25);
  }

  // function to climb down -- motor backwards direction
  private void climbDown() {
    // motor.set(-0.25);
    move(MIN_DISTANCE);
    fLogger.log("Climb down ");
  }

  private void stop() {
    motor.setControl(stop);
    fLogger.log("Stop ");
  }

  /**
   * This command runs the climb up action
   * by extending the climber arm.
   * 
   * @return Command
   */
  public Command runClimbUp() {
    Command cmd = runOnce(() -> climbUp())
        .andThen(Commands.waitUntil(() -> isClimberAtTarget()))
        .withName("Climb Up");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /**
   * This command runs the climb down action
   * by retracting the climber arm.
   * 
   * @return Command
   */
  public Command runClimbDown() {
    Command cmd = runOnce(() -> climbDown())
        .andThen(Commands.waitUntil(() -> isClimberAtTarget()))
        .withName("Climb Down");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /**
   * This command runs the stop action.
   * 
   * @return Command
   */
  public Command runStop() {
    Command cmd = runOnce(() -> stop())
        .andThen(Commands.waitUntil(() -> isClimberAtTarget()))
        .withName("Climb Stop");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  // grabber motors

  private void openClaw() {
    grabber.setControl(grabberPosition.withPosition(MAX_GRABBER_DISTANCE));
    fLogger.log("Open Claw ");
  }

  private void closeClaw() {
    grabber.setControl(grabberPosition.withPosition(MIN_GRABBER_POSITION));
  }

  private void stopClaw() {
    grabber.setControl(stop);
  }

  /**
   * This command runs the open claw action.
   * 
   * @return Command
   */
  public Command runOpenClaw() {
    Command cmd = runOnce(() -> openClaw())
        .andThen(Commands.waitUntil(() -> isGrabberAtTarget()))
        .withName("Open Claw");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /**
   * This command runs the close claw action.
   * 
   * @return Command
   */
  public Command runCloseClaw() {
    Command cmd = runOnce(() -> closeClaw())
        .andThen(Commands.waitUntil(() -> isGrabberAtTarget()))
        .withName("Close Claw");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /**
   * This command runs the stop claw action.
   * 
   * @return Command
   */
  public Command runStopClaw() {
    Command cmd = runOnce(() -> stopClaw())
        .andThen(Commands.waitUntil(() -> isClimberAtTarget()))
        .withName("Stop Claw");
    SmartDashboard.putData(cmd);
    return cmd;
  }

  /**
   * This command will extend the climber and then open the claw.
   * 
   * @return
   */
  public Command runExtendClimber() {
    return runClimbUp().andThen(runOpenClaw()).withName("Extend Climber");
  }

  public Command runRetractClimber() {
    return runCloseClaw().andThen(runClimbDown()).withName("Retract Climber");
  }

  private static final double MAX_POSITION = 5.1;
  private static final double MAX_GRABBER_POSITION = 3.0;
  private static final double MIN_POSITION = -5.1;
  private static final double MIN_GRABBER_POSITION = 0.0;

  public void checkLimits() {
    if (motor.getPosition().getValueAsDouble() >= MAX_POSITION) {
      stop();
    }
    if (grabber.getPosition().getValueAsDouble() >= MAX_GRABBER_POSITION) {
      stopClaw();
    }
    if (motor.getPosition().getValueAsDouble() <= MIN_POSITION) {
      stop();
    }
    if (grabber.getPosition().getValueAsDouble() <= MIN_GRABBER_POSITION) {
      stopClaw();
    }
  }
  // position limits

}
