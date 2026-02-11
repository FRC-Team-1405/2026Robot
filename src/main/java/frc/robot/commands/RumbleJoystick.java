// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Give a general description
 */
public class RumbleJoystick extends SequentialCommandGroup {
  /**
   * Give a description of the class
   * 
   * @param runRumbleStart(joystick,_RumbleType.kBothRumble,_1.0),
   * @param new_WaitCommand(2.0),
   * @param runRumbleStop(joystick));
   * 
   */
  public RumbleJoystick(CommandXboxController joystick, RumbleType rumbleType, double rumbleStrength) {
    addCommands(
        runRumbleStart(joystick, RumbleType.kBothRumble, 1.0),
        new WaitCommand(2.0),
        runRumbleStop(joystick));
  }

  /**
   * Rumble for 3 seconds.
   * 
   * @param joystick
   */
  public RumbleJoystick(CommandXboxController joystick) {
    this(joystick, RumbleType.kBothRumble, 3.0);
  }

  private Command runRumbleStart(CommandXboxController joystick, RumbleType rumbleType, double rumbleStrength) {
    return Commands.runOnce(() -> {
      joystick.setRumble(rumbleType, rumbleStrength);
    });
  }

  public Command runRumbleStop(CommandXboxController joystick) {
    return runRumbleStart(joystick, RumbleType.kBothRumble, 0.0);
  }
}
