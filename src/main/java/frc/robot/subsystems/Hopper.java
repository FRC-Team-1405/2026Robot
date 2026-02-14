// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.FinneyLogger;
import frc.robot.sim.SimProfiles;

public class Hopper extends SubsystemBase {
  private final FinneyLogger fLogger = new FinneyLogger(this.getClass().getSimpleName());
  /** Creates a new Hopper. */
  private TalonFX motor = new TalonFX(Constants.CANBus.HOPPER_MOTOR);
  private final VelocityVoltage speed = new VelocityVoltage(0);
  private final NeutralOut stop = new NeutralOut();

  public Hopper() {
    SimProfiles.initHopper(motor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(AngularVelocity velocity) {
    motor.setControl(speed.withVelocity(velocity));
  }

  private void forwardHopper() {
    setSpeed(Constants.HopperPreferences.HOPPER_FORWARD_SPEED);
    fLogger.log("Forward Hopper ");
  }

  private void reverseHopper() {
    setSpeed(Constants.HopperPreferences.HOPPER_REVERSE_SPEED);
    fLogger.log("Reverse Hopper ");
  }

  private void stopHopper() {
    motor.setControl(stop);
    fLogger.log("Stop Hopper ");
  }

  public Command runForwardHopper() {
    return startEnd(() -> forwardHopper(), () -> stopHopper()).withName("Run Forward Hopper");
  }

  public Command runReverseHopper() {
    return startEnd(() -> reverseHopper(), () -> stopHopper()).withName("Run Reverse Hopper");
  }
}
