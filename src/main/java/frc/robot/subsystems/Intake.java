// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FinneyLogger;
import frc.robot.sim.SimProfiles;

public class Intake extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());

  private TalonFX intakeMotor = new TalonFX(Constants.CANBus.INTAKE_MOTOR);
  private final MotionMagicVoltage intakeMotorSpeed = new MotionMagicVoltage(0);
  private final NeutralOut stop = new NeutralOut();
  private TalonFX pickupMotor = new TalonFX(Constants.CANBus.PICKUP_MOTOR);
  private final VelocityVoltage pickupMotorSpeed = new VelocityVoltage(0);
  /** Creates a new Intake. */
  private int intakeSettleCount = 0;
  private double intakePositionTarget = 0;
  private boolean isIntakeDeployed = false;
  private boolean isPickupActive = false;

  public Intake() {
    SimProfiles.initIntake(intakeMotor);
    SimProfiles.initPickup(pickupMotor);
  }

  @Override
  public void periodic() {
    if (Math.abs(intakePositionTarget
        - intakeMotor.getClosedLoopReference().getValueAsDouble()) < Constants.IntakePreferences.POSITION_TOLERANCE) {
      if (Math
          .abs(intakeMotor.getClosedLoopError().getValueAsDouble()) < Constants.IntakePreferences.POSITION_TOLERANCE) {
        intakeSettleCount += 1;
      } else {
        intakeSettleCount = 0;
      }
    } else {
      intakeSettleCount = 0;
    }
  }

  private boolean isIntakeAtTarget() {
    return (intakeSettleCount >= Constants.IntakePreferences.SETTLE_MAX);
  }

  private void moveIntake(double position) {
    intakeMotor.setControl(intakeMotorSpeed.withPosition(position));
    intakePositionTarget = position;
  }

  private void movePickup(double speed) {
    pickupMotor.setControl(pickupMotorSpeed.withVelocity(speed));
    isPickupActive = true;
  }

  private void intakeOut() {
    isIntakeDeployed = true;
    moveIntake(Constants.IntakePreferences.INTAKE_MOTOR_OUT);
    fLogger.log("Intake Out ");
  }

  private void intakeIn() {
    isIntakeDeployed = false;
    moveIntake(Constants.IntakePreferences.INTAKE_MOTOR_IN);
    fLogger.log("Intake In ");
  }

  private void intakeCenter() {
    isIntakeDeployed = true;
    moveIntake(Constants.IntakePreferences.INTAKE_MOTOR_CENTER);
    fLogger.log("Intake Center");
  }

  private void pickupOut() {
    movePickup(Constants.IntakePreferences.PICKUP_MOTOR_OUT);
    fLogger.log("Pickup Out ");
  }

  private void pickupIn() {
    movePickup(Constants.IntakePreferences.PICKUP_MOTOR_IN);
    fLogger.log("Pickup In ");
  }

  private void stopIntake() {
    intakeMotor.setControl(stop);
    fLogger.log("Stop Intake ");
  }

  private void stopPickup() {
    pickupMotor.setControl(stop);
    isPickupActive = false;
    fLogger.log("Stop Pickup ");
  }

  public Command runIntakeOut() {
    return Commands.sequence(
        runOnce(() -> intakeOut()),
        Commands.waitUntil(() -> isIntakeAtTarget()))
        .finallyDo(() -> stopIntake())
        .withName("Run Intake Out");
  }

  public Command runIntakeIn() {
    return Commands.sequence(
        runOnce(() -> intakeIn()),
        Commands.waitUntil(() -> isIntakeAtTarget()))
        .finallyDo(() -> stopIntake())
        .withName("Run Intake In");
  }

  public Command runIntakeCenter() {
    return Commands.sequence(
        runOnce(() -> intakeCenter()),
        Commands.waitUntil(() -> isIntakeAtTarget()))
        .finallyDo(() -> stopIntake())
        .withName("Run Intake Center");
  }

  public Command runPickupOut(String name) {
    Command cmd = runPickupOut().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  public Command runPickupOut() {
    return runOnce(() -> pickupOut());
  }

  public Command runPickupIn(String name) {
    Command cmd = runPickupIn().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  public Command runPickupIn() {
    return runOnce(() -> pickupIn());
  }

  public Command runPickupStop(String name) {
    Command cmd = runPickupStop().withName(name);
    SmartDashboard.putData(cmd);
    return cmd;
  }

  public Command runPickupStop() {
    return runOnce(() -> stopPickup());
  }

  // public Command runPickupFuel() {
  // return Commands.sequence(
  // runIntakeOut(),
  // runPickupIn());
  // }

  public Command runRetractIntake() {
    return Commands.sequence(
        runPickupStop(),
        runIntakeIn());
  }

  public boolean isIntakeExtended() {
    return isIntakeDeployed;
  }

  public boolean isPickupRunning() {
    return isPickupActive;
  }

}
