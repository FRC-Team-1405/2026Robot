package frc.robot.lib;

import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoPilot.AutoPilotCommands;
import frc.robot.commands.PidToPose.PidToPoseCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCommands {
    private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static final String AUTO_SMARTDASHBOARD_FOLDER = "Auto";

    public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber) {
        SmartDashboard.putBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false);
        SmartDashboard.putData(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode", autoChooser);

        PidToPoseCommands.registerCommands(drivetrain);
        AutoPilotCommands.registerCommands(drivetrain, climber);
        // this HAS to go after AutoPilotCommands
        AutoCommands.configureAutos(autoChooser, drivetrain);

        NamedCommands.registerCommand("Left - [My First Auto]",
                Commands.print("Running [My First Auto] from the LEFT"));

    }

    public static void configureAutos(SendableChooser<Command> chooser, CommandSwerveDrivetrain drivetrain) {
        HashMap<String, Command> commandsToAddToChooser = new HashMap<>();

        // region PidToPose
        // commandsToAddToChooser.put("P2P_auto1",
        // NamedCommands.getCommand("P2P_auto1"));
        // endregion PidToPose

        commandsToAddToChooser.put("AP_FrontHubShoot", NamedCommands.getCommand("AP_FrontHubShoot"));

        commandsToAddToChooser.put("AP_blueCenterToDepot", NamedCommands.getCommand("AP_blueCenterToDepot"));
        commandsToAddToChooser.put("AP_DepotFaceIn", NamedCommands.getCommand("AP_DepotFaceIn"));
        commandsToAddToChooser.put("AP_climb", NamedCommands.getCommand("AP_climb"));
        commandsToAddToChooser.put("AP_JUSTSHOOT", NamedCommands.getCommand("AP_JUSTSHOOT"));
        commandsToAddToChooser.put("AP_rightBumpToField", NamedCommands.getCommand("AP_rightBumpToField"));
        commandsToAddToChooser.put("AP_leftBumpToField", NamedCommands.getCommand("AP_leftBumpToField"));
        commandsToAddToChooser.put("AP_rightBumpToAlliance", NamedCommands.getCommand("AP_rightBumpToAlliance"));
        commandsToAddToChooser.put("AP_leftBumpToAlliance", NamedCommands.getCommand("AP_leftBumpToAlliance"));

        commandsToAddToChooser.put("AP_ShootFromDepot", NamedCommands.getCommand("AP_ShootFromDepot"));

        commandsToAddToChooser.put("AP_CenterHarvest", NamedCommands.getCommand("AP_CenterHarvest"));
        commandsToAddToChooser.put("AP_LeftStartDepotScore", NamedCommands.getCommand("AP_LeftStartDepotScore"));
        commandsToAddToChooser.put("AP_RightStartDepotScore", NamedCommands.getCommand("AP_RightStartDepotScore"));
        commandsToAddToChooser.put("AP_RightStartFeedingStationScore",
                NamedCommands.getCommand("AP_RightStartFeedingStationScore"));
        commandsToAddToChooser.put("AP_TheShowboater", NamedCommands.getCommand("AP_TheShowboater"));
        commandsToAddToChooser.put("AP_fourMeters", NamedCommands.getCommand("AP_fourMeters"));
        commandsToAddToChooser.put("AP_RightStartCenterHarvestInLeft",
                NamedCommands.getCommand("AP_RightStartCenterHarvestInLeft"));
        commandsToAddToChooser.put("AP_RightFeedShootCenterHarvestInLeftShoot",
                NamedCommands.getCommand("AP_RightFeedShootCenterHarvestInLeftShoot"));
        commandsToAddToChooser.put("AP_LeftDepotShootCenterHarvestInLeftShoot",
                NamedCommands.getCommand("AP_LeftDepotShootCenterHarvestInLeftShoot"));

        // endregion autopilot
        // commandsToAddToChooser.put("Left_First_Auto", NamedCommands.getCommand("Left
        // - [My First Auto]"));

        // chooser.setDefaultOption("P2P_auto1",
        // commandsToAddToChooser.get("P2P_auto1"));

        // Add all commands in Map to chooser
        commandsToAddToChooser.keySet().stream()
                .forEach(name -> chooser.addOption(name, commandsToAddToChooser.get(name)));

        initShootPositions();

    }

    public static SendableChooser<Supplier<Pose2d>> shootingPositions = new SendableChooser<>();

    public static void initShootPositions() {
        shootingPositions.addOption("FrontHub", AutoPilotCommands.FrontHubShoot);
        shootingPositions.setDefaultOption("FrontHub", AutoPilotCommands.FrontHubShoot);
        SmartDashboard.putData(AUTO_SMARTDASHBOARD_FOLDER + " Shooting Position", shootingPositions);
    }

    public static Supplier<Pose2d> getShootPosition() {
        return shootingPositions.getSelected();
        // SmartDashboard.getString(AUTO_SMARTDASHBOARD_FOLDER + "shooterPositions",
        // NamedCommands.getCommand("AP_FrontHubShoot"));

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