// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AdjustableHood extends SubsystemBase {
  private final Servo servo = new Servo(0);
  private double position = 0.0;
  private double target = 0.0;
  private long lastTime = System.nanoTime();

  /** Creates a new AdjustableHood. */
  /**
   * Hood that adjusts angle using a servo.
   */
  public AdjustableHood() {

  }

  @Override
  public void periodic() {
    long currentTime = System.nanoTime();
    long elapsedTime = currentTime - lastTime;
    double distance = Constants.HoodPreferences.SERVO_SPEED_SECONDS * (elapsedTime / 1e9);
    if (target > position) {
      position = Math.min(position + distance, target);
    } else if (target < position) {
      position = Math.max(position - distance, target);
    }
    lastTime = currentTime;

    SmartDashboard.putNumber("Hood Position", position);
    SmartDashboard.putNumber("Hood Target", target);
  }

  private boolean isAtTarget() {
    return Math.abs(position - target) < 0.01;
  }

  // private function to set servo position (double)
  private void setServo(double position) {
    target = position;
    servo.set(position);
  }

  /**
   * Sets the servo and then waits for the hood to reach the position.
   * 
   * @param position
   * @return
   */
  public Command runSet(double position) {
    return runOnce(() -> setServo(position)).andThen(Commands.waitUntil(this::isAtTarget));
  }
}
