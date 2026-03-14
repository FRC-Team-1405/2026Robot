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
import frc.robot.commands.AutoPilot.CommandsForAutoPilot;
import frc.robot.commands.PidToPose.PidToPoseCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoCommands {
        private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

        private static final String AUTO_SMARTDASHBOARD_FOLDER = "Auto";

        public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber, Intake intake,
                        Hopper hopper,
                        Indexer indexer,
                        Shooter shooter) {
                SmartDashboard.putBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false);
                SmartDashboard.putData(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode", autoChooser);

                PidToPoseCommands.registerCommands(drivetrain);
                CommandsForAutoPilot.registerCommands(drivetrain, climber, intake, hopper, indexer, shooter);
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

                commandsToAddToChooser.put("FrontHubShoot", NamedCommands.getCommand("FrontHubShoot"));

                // commandsToAddToChooser.put("blueCenterToDepot",
                // NamedCommands.getCommand("blueCenterToDepot"));
                // commandsToAddToChooser.put("DepotFaceIn",
                // NamedCommands.getCommand("DepotFaceIn"));
                // commandsToAddToChooser.put("climb", NamedCommands.getCommand("climb"));

                commandsToAddToChooser.put("JUSTSHOOT", NamedCommands.getCommand("JUSTSHOOT"));
                commandsToAddToChooser.put("LeftStart_JUSTSHOOT", NamedCommands.getCommand("LeftStart_JUSTSHOOT"));

                // TODO:BUMP AUTOS
                // commandsToAddToChooser.put("rightBumpToField",
                // NamedCommands.getCommand("rightBumpToField"));
                // commandsToAddToChooser.put("leftBumpToField",
                // NamedCommands.getCommand("leftBumpToField"));
                // commandsToAddToChooser.put("rightBumpToAlliance",
                // NamedCommands.getCommand("rightBumpToAlliance"));
                // commandsToAddToChooser.put("leftBumpToAlliance",
                // NamedCommands.getCommand("leftBumpToAlliance"));

                // commandsToAddToChooser.put("ShootFromDepot",
                // NamedCommands.getCommand("ShootFromDepot"));
                // commandsToAddToChooser.put("CenterHarvest",
                // NamedCommands.getCommand("CenterHarvest"));
                // commandsToAddToChooser.put("LeftStartDepotScore",
                // NamedCommands.getCommand("LeftStartDepotScore"));
                // commandsToAddToChooser.put("RightStartDepotScore",
                // NamedCommands.getCommand("RightStartDepotScore"));
                // commandsToAddToChooser.put("TheShowboater",
                // NamedCommands.getCommand("TheShowboater"));

                commandsToAddToChooser.put("TEST", NamedCommands.getCommand("TEST"));

                // Feeding station
                commandsToAddToChooser.put("RightStartFeedingStationScore",
                                NamedCommands.getCommand("RightStartFeedingStationScore"));
                commandsToAddToChooser.put("CenterStartFeedingStationScore",
                                NamedCommands.getCommand("CenterStartFeedingStationScore"));
                commandsToAddToChooser.put("LeftStartFeedingStationScore",
                                NamedCommands.getCommand("LeftStartFeedingStationScore"));

                // NamedCommands.registerCommand("RightStartFeedingStationScore",
                // RightStartFeedingStationScore);
                // NamedCommands.registerCommand("CenterStartFeedingStationScore",
                // CenterStartFeedingStationScore);
                // NamedCommands.registerCommand("LeftStartFeedingStationScore",
                // LeftStartFeedingStationScore);

                commandsToAddToChooser.put("RightStartCenterHarvestInLeft",
                                NamedCommands.getCommand("RightStartCenterHarvestInLeft"));
                commandsToAddToChooser.put("LeftStartCenterHarvestInRight",
                                NamedCommands.getCommand("LeftStartCenterHarvestInRight"));

                // commandsToAddToChooser.put("LeftDepotShootCenterHarvestInLeftShoot",
                // NamedCommands.getCommand("LeftDepotShootCenterHarvestInLeftShoot"));
                commandsToAddToChooser.put("RightQuad",
                                NamedCommands.getCommand("RightQuad"));
                commandsToAddToChooser.put("LeftQuad",
                                NamedCommands.getCommand("LeftQuad"));

                // commandsToAddToChooser.put("Right_Yum_Middle",
                // NamedCommands.getCommand("Right_Yum_Middle"));

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
                shootingPositions.addOption("FrontHub", CommandsForAutoPilot.FrontHubShoot);
                shootingPositions.addOption("IntakeIN_FrontHub", CommandsForAutoPilot.IntakeIN_FrontHubShoot);
                shootingPositions.addOption("IntakeOUT_FrontHub", CommandsForAutoPilot.IntakeOUT_FrontHubShoot);
                shootingPositions.addOption("RightHub", CommandsForAutoPilot.RightHubShoot);

                shootingPositions.setDefaultOption("FrontHub", CommandsForAutoPilot.FrontHubShoot);
                SmartDashboard.putData(AUTO_SMARTDASHBOARD_FOLDER + " Shooting Position", shootingPositions);
        }

        public static Supplier<Pose2d> getShootPosition() {
                return shootingPositions.getSelected();
                // SmartDashboard.getString(AUTO_SMARTDASHBOARD_FOLDER + "shooterPositions",
                // NamedCommands.getCommand("FrontHubShoot"));

        }

        public static Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                if (DriverStation.isFMSAttached()
                                || SmartDashboard.getBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false)) {
                        SmartDashboard.putBoolean(AUTO_SMARTDASHBOARD_FOLDER + "/Auto Mode Enable", false);
                        if (CommandsForAutoPilot.OVERRIDE_AUTO_COMMAND != null) {
                                return CommandsForAutoPilot.OVERRIDE_AUTO_COMMAND;
                        }
                        return autoChooser.getSelected();
                } else {
                        return Commands.print("Auto Disabled");
                }

        }
}