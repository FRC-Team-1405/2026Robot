package frc.robot.lib;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoPilot.AutoPilotCommands;
import frc.robot.commands.PidToPose.PidToPoseCommands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCommands {
    private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static final String AUTO_SMARTDASHBOARD_FOLDER = "Auto";

    public static void registerCommands(CommandSwerveDrivetrain drivetrain) {
        SmartDashboard.putBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false);
        SmartDashboard.putData(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode", autoChooser);

        PidToPoseCommands.registerCommands(drivetrain);
        AutoPilotCommands.registerCommands(drivetrain);

        AutoCommands.configureAutos(autoChooser, drivetrain);

        NamedCommands.registerCommand("Left - [My First Auto]",
                Commands.print("Running [My First Auto] from the LEFT"));

    }

    public static void configureAutos(SendableChooser<Command> chooser, CommandSwerveDrivetrain drivetrain) {
        HashMap<String, Command> commandsToAddToChooser = new HashMap<>();

        // region PidToPose
        commandsToAddToChooser.put("P2P_auto1", NamedCommands.getCommand("P2P_auto1"));
        // endregion PidToPose

        // region autopilot
        commandsToAddToChooser.put("AP_auto1", NamedCommands.getCommand("AP_auto1"));
        commandsToAddToChooser.put("AP_tracking_example", NamedCommands.getCommand("AP_tracking_example"));
        // endregion autopilot
        commandsToAddToChooser.put("Left_First_Auto", NamedCommands.getCommand("Left - [My First Auto]"));

        chooser.setDefaultOption("P2P_auto1", commandsToAddToChooser.get("P2P_auto1"));

        // Add all commands in Map to chooser
        commandsToAddToChooser.keySet().stream()
                .forEach(name -> chooser.addOption(name, commandsToAddToChooser.get(name)));
    }

    public static Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        if (DriverStation.isFMSAttached()
                || SmartDashboard.getBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false)) {
            SmartDashboard.putBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false);
            return autoChooser.getSelected();
        } else {
            return Commands.print("Auto Disabled");
        }
    }
}
