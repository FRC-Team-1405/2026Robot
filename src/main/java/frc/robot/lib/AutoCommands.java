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
    }

    public static void configureAutos(SendableChooser<Command> chooser, CommandSwerveDrivetrain drivetrain) {
        HashMap<String, Command> commandsToAddToChooser = new HashMap<>();

        // region PidToPose
        commandsToAddToChooser.put("P2P_auto1", NamedCommands.getCommand("P2P_auto1"));
        // endregion PidToPose

        // region autopilot
        commandsToAddToChooser.put("AP_auto1", NamedCommands.getCommand("AP_auto1"));
        commandsToAddToChooser.put("AP_tracking_example", NamedCommands.getCommand("AP_tracking_example"));
        commandsToAddToChooser.put("AP_blueCenter", NamedCommands.getCommand("AP_blueCenter"));
        commandsToAddToChooser.put("AP_blueCenterToDepot", NamedCommands.getCommand("AP_blueCenterToDepot"));
        commandsToAddToChooser.put("AP_blueCenterToOriginToRightBump",
                NamedCommands.getCommand("AP_blueCenterToOriginToRightBump"));
        commandsToAddToChooser.put("AP_blueScoreBumpFeed", NamedCommands.getCommand("AP_blueScoreBumpFeed"));
        commandsToAddToChooser.put("AP_FEEDME", NamedCommands.getCommand("AP_FEEDME"));
        commandsToAddToChooser.put("AP_origin", NamedCommands.getCommand("AP_origin"));

        // endregion autopilot

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
