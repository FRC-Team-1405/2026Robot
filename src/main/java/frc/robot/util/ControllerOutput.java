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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControllerOutput {
    public void startVibration(CommandXboxController joystick) {
        joystick.setRumble(RumbleType.kBothRumble, 1.0);
        System.out.println("x button pressed");
    }

}
