// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import java.lang.Math;
import java.security.spec.DSAPrivateKeySpec;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Prefs;
import frc.robot.sim.sjc.MotorSim_Mech_SJC;
import frc.robot.sim.sjc.PhysicsSim_SJC;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterMotor1 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.CANBus.SHOOTER_MOTOR_2);

  private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_Brake = new NeutralOut();

  private int settleCount = 0;

  private boolean locked = false;

  private final MotorSim_Mech_SJC shooterMotorSimMech = new MotorSim_Mech_SJC("Shooter/FlywheelViz");

  public boolean isReadyToFire() {
    return locked;
  }

  public void setShooterMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kV = 0.05;
    configs.Slot0.kS = 0.0;

    configs.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterMotor1.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());

      shooterMotor1.setControl(m_Brake);
    }

  }

  private void setShooterSpeed(Supplier<AngularVelocity> speed) {
    shooterMotor1.setControl(m_VelocityVoltage.withVelocity(speed.get()));
    settleCount = 0;
    locked = false;
  }

  private void shooterStop() {
    shooterMotor1.setControl(m_Brake);
  }

  /** Creates a new Shooter. */
  public Shooter() {
    simulationInit();
    shooterMotor2.setControl(new Follower(Constants.CANBus.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
    // SimProfiles.initShooter(shooterMotor1);
    stopShooter();
    // setShooterMotor();
  }

  public Command runShooter(Supplier<AngularVelocity> speed) {
    return Commands.runOnce(() -> setShooterSpeed(speed), this);
  };

  public Command stopShooter() {
    return Commands.runOnce(() -> shooterStop(), this);
  }

  @Override
  public void periodic() {
    // TODO update to use velocity signal for higher refresh rate
    shooterMotorSimMech.update(shooterMotor1.getPosition(), shooterMotor1.getVelocity());

    double error = shooterMotor1.getClosedLoopError().getValueAsDouble();
    double range = locked ? Prefs.WIDE : Prefs.TIGHT; // TODO improve name on "locked"
    if (Math.abs(error) < range) {
      if (settleCount < Prefs.STABLE_COUNT) {
        settleCount += 1;
      }
    } else {
      settleCount = 0;
    }

    locked = settleCount >= Prefs.STABLE_COUNT;

    SmartDashboard.putNumber("Shooter/RPS", shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/CurrentDraw", shooterMotor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Error", shooterMotor1.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/SettleCount", settleCount);
    SmartDashboard.putBoolean("Shooter/Locked", locked);
  }

  //
  // Simulation code
  //

  /**
   * Initialize simulation components
   */
  public void simulationInit() {
    if (!edu.wpi.first.wpilibj.RobotBase.isReal()) {
      // Add leader motor with gearbox simulation
      // Parameters: motor, rotorInertia, loadMass, armLength, viscousCoeff,
      // numMotors, gearRatio
      // - viscousCoeff: air resistance on spinning flywheel (~0.001 for enclosed
      // flywheel)
      // - numMotors: 2 (both motors driving same gearbox)
      double flywheelMassKg = Pounds.of(Constants.Prefs.FLYWHEEL_WEIGHT_LBS).in(Kilograms);
      double flywheelRadiusMeters = Inches.of(Constants.Prefs.FLYWHEEL_DIAMETER_INCHES / 2.0).in(Meters);
      double viscousDamping = 0.001; // Light air resistance

      PhysicsSim_SJC.getInstance().addTalonFX(
          shooterMotor1,
          Constants.Prefs.FLYWHEEL_MOMENT_OF_INERTIA,
          flywheelMassKg,
          flywheelRadiusMeters,
          viscousDamping,
          2, // numberOfMotors in gearbox
          Constants.Prefs.MOTOR_TO_WHEEL_GEAR_RATIO);
    }
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim_SJC.getInstance().run();
  }

}
