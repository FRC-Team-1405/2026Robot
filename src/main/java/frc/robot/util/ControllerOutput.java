// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.annotation.ObjectIdGenerator.IdKey;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControllerOutput {
    //
    // Not Needed better code is in RumbleJoystick.java
    //
    // public ControllerOutput(CommandXboxController myjoystick) {
    // joystick = myjoystick;
    // }
    //
    // public CommandXboxController joystick;
    //
    // public SequentialCommandGroup gamePeriodChangeGroup = new
    // SequentialCommandGroup(
    // startRumble(joystick, RumbleType.kBothRumble, 1.0),
    // new WaitCommand(2.0),
    // stopRumble(joystick));
    //
    // public Command gamePeriodChange(CommandXboxController joystick) {
    // return Commands.runOnce(() -> {
    // startRumble(joystick, RumbleType.kBothRumble, 1.0);
    // });
    // }
    //
    // public Command stopRumble(CommandXboxController joystick) {
    //
    // return startRumble(joystick, RumbleType.kBothRumble, 0.0);
    // }
    //
    // public Command startRumble(CommandXboxController joystick, RumbleType
    // rumbleType, double rumbleStrength) {
    // return Commands.runOnce(() -> {
    // joystick.setRumble(rumbleType, rumbleStrength);
    // });
    // }
}
