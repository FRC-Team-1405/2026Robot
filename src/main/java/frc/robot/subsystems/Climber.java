// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.FinneyLogger;
import frc.robot.lib.MotorSim.MotorSim_Mech;
import frc.robot.lib.MotorSim.PhysicsSim;
import frc.robot.sim.SimProfiles;

public class Climber extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());
  /** Creates a new Climber. */
  private TalonFX motor = new TalonFX(45);
  private TalonFX grabber = new TalonFX(37);
  private MotorSim_Mech Climber_motorSimMech = new MotorSim_Mech("ClimberMotorSimMech");

  private static final double CLIMBER_CURRENT_LIMIT = 40.0;

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Climber Position", getMotorPos());
    Climber_motorSimMech.update(motor.getPosition(), motor.getVelocity());
    SmartDashboard.putNumber("Climber Voltage", motor.getMotorVoltage().getValueAsDouble());
    fLogger.log("Motor voltage = " + motor.getMotorVoltage().getValueAsDouble() + " Motor position = "
        + motor.getPosition().getValueAsDouble());
    if (Utils.isSimulation()) {
      // motor.setPosition(motor.getPosition().getValueAsDouble() +
      // motor.getMotorVoltage().getValueAsDouble());
      // set motor position to ( motor position as decimal )
      // set motor position to ( current position )
      // set motor position = current position + motor voltage
    }
    // System.out.println("Claw motor voltage = " +
    // grabber.getMotorVoltage().getValueAsDouble() + " Claw motor position"
    // + grabber.getPosition().getValueAsDouble());
    if (Utils.isSimulation()) {
      grabber.setPosition(grabber.getPosition().getValueAsDouble() + grabber.getMotorVoltage().getValueAsDouble());
    }

    // if (motor.getSupplyCurrent().getValueAsDouble() > CLIMBER_CURRENT_LIMIT) {
    // stop();
    // fLogger.log("CLimber current trip " +
    // motor.getSupplyCurrent().getValueAsDouble());
    // }
    // safety stop

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

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(motor, 0.001);// , 0.0, 0.0, 0.0, 2, 1.0);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  private double MAX_DISTANCE = 5.1;
  private double MIN_DISTANCE = -5.1;
  private double STOP_CLIMBER = 0.0;

  public void move(double position) {
    // position = MAX_DISTANCE * (position);
    // motor.setControl(new MotionMagicVoltage(position));
    motor.set(0.5);
  }

  public void moveGrabber(double position) {
    // motor.setControl(new MotionMagicVoltage(position));
    motor.set(0.2);
  }

  public Climber() {
    SimProfiles.initClimber(motor);
    setupMotors();
  }

  // function to climb up - motor forward direction
  public void climbUp() {
    move(MAX_DISTANCE);
    fLogger.log("Climb up ");
    // motor.set(0.25);
  }

  // function to climb down -- motor backwards direction
  public void climbDown() {
    // motor.set(-0.25);
    move(MIN_DISTANCE);
    fLogger.log("Climb down ");
  }

  public void stop() {
    move(STOP_CLIMBER);
    fLogger.log("Stop ");
  }

  // grabber motors

  public void openClaw() {
    grabber.set(0.05);
  }

  public void closeClaw() {
    grabber.set(-0.05);
  }

  public void stopClaw() {
    grabber.set(0);
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
