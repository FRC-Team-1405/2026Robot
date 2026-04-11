// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodPreferences;
import frc.robot.Constants.HoodPreferences.HoodAngles;
import frc.robot.subsystems.AdjustableHood;

public class SetHoodPosition extends Command {

  private final AdjustableHood subsystem;
  private final DoubleSupplier targetPosition;
  private double moveTimeSeconds;

  private HoodAngles hoodAngle;

  private double startTime;

  /**
   * @param subsystem       the servo subsystem
   * @param targetPosition  servo position 0.0–1.0
   * @param moveTimeSeconds safe wait time (0.3–0.6 typical)
   */
  public SetHoodPosition(AdjustableHood subsystem, DoubleSupplier targetPosition) {
    this.subsystem = subsystem;
    this.targetPosition = targetPosition;

    hoodAngle = null;
    SmartDashboard.putString("Hood/Hood Position", HoodAngles.ZERO.name());

    addRequirements(subsystem);
  }

  public SetHoodPosition(AdjustableHood subsystem, HoodAngles angle) {
    this(subsystem, () -> {
      return angle.getPositionPercentage();
    });
    this.hoodAngle = angle;
  }

  @Override
  public void initialize() {
    double target = targetPosition.getAsDouble();
    double pos = subsystem.getTarget();
    moveTimeSeconds = Math.abs(targetPosition.getAsDouble() - subsystem.getTarget())
        * HoodPreferences.SERVO_FULL_RANGE_SECONDS;
    subsystem.setServo(targetPosition.getAsDouble());
    startTime = Timer.getFPGATimestamp();

    if (hoodAngle != null) {
      SmartDashboard.putString("Hood/Hood Position", hoodAngle.name());
    }
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= moveTimeSeconds;
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopServo();
  }
}
