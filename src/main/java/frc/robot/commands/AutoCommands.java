package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoPilot.AutoPilotCommands;
import frc.robot.commands.PidToPose.PidToPoseCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCommands {
    public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
        PidToPoseCommands.registerCommands(drivetrain);
        AutoPilotCommands.registerCommands(drivetrain);
    }

    public static void configureAutos(SendableChooser<Command> chooser, CommandSwerveDrivetrain drivetrain) {
        HashMap<String, Command> commandsToAddToChooser = new HashMap<>();

        // region PidToPose
        commandsToAddToChooser.put("P2P_auto1", NamedCommands.getCommand("P2P_auto1"));
        // endregion PidToPose

        // region autopilot
        commandsToAddToChooser.put("AP_auto1", NamedCommands.getCommand("AP_auto1"));
        // endregion autopilot

        chooser.setDefaultOption("P2P_auto1", commandsToAddToChooser.get("P2P_auto1"));

        // Add all commands in Map to chooser
        commandsToAddToChooser.keySet().stream()
                .forEach(name -> chooser.addOption(name, commandsToAddToChooser.get(name)));
    }
}
