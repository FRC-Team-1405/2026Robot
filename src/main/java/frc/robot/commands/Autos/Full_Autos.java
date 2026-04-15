// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Full_Autos {
        private CommandsForAutos cmds;
        public static Command OVERRIDE_AUTO_COMMAND = null;

        public Full_Autos(CommandsForAutos cmds) {
                this.cmds = cmds;
        }

        Command FrontHubShoot = new SequentialCommandGroup(
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get());
        Command LeftStart_ToDepot = new SequentialCommandGroup(
                        // Commands.parallel(MoveTo_leftBump_AllianceToFieldStart.get(),
                        // intake.runIntakeOut()),
                        Commands.parallel(
                                        cmds.intake.runIntakeOut(),
                                        cmds.MoveTo_leftBump_AllianceToFieldStart.get()),
                        cmds.MoveTo_depot_BackFace_Start.get(),
                        Commands.deadline(
                                        Commands.sequence(
                                                        cmds.MoveTo_depot_BackFace_End.get(),
                                                        cmds.MoveTo_depot_BackFace_End_Dos.get(),
                                                        cmds.MoveTo_midOfDepotFaceOut.get()),
                                        cmds.intake.runPickupIn()),

                        cmds.MoveTo_depot_BackFace_Start.get(),

                        // intake.runPickupStop(),
                        cmds.MoveTo_New_FrontHubShoot.get());

        Command TEST = new SequentialCommandGroup(

                        cmds.MoveTo_depot_BackFace_Start.get(),
                        cmds.intake.runIntakeOut(),
                        cmds.MoveTo_depot_BackFace_End.get(),
                        cmds.MoveTo_depot_BackFace_Start.get(),
                        cmds.MoveTo_New_FrontHubShoot.get());
        Command JUSTSHOOT = new SequentialCommandGroup(
                        // MoveTo_allianceCenter.get(),
                        // MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.intake.runIntakeOut(),
                        // intake.runPickupIn().withTimeout(0.5),
                        Commands.parallel(cmds.MoveTo_IntakeIN_FrontHubShoot.get(),
                                        cmds.intake.runPickupIn().withTimeout(3.0)),
                        cmds.intake.runIntakeCenter(),
                        // MoveTo_depot_BackFace_End.get(),
                        cmds.MoveTo_IntakeOUT_FrontHubShoot.get()

        // MoveTo_rightBump_AllianceToFieldStart.get(),
        // MoveTo_rightBump_AllianceToFieldEnd.get(),
        // MoveTo_centerRightIntakeStart.get(),
        // MoveTo_centerRightIntakeEnd.get(),
        // MoveTo_leftLoadInZone.get()
        // MoveTo_rightLoadInZone.get()
        );

        Command JUST_SHOOT_FROM_ANYWHERE = Commands.sequence(cmds.MoveTo_ClosestShootingPosition_SHORT.get(),
                        cmds.shortShoot.get());

        Command LeftStart_JUSTSHOOT = new SequentialCommandGroup(
                        cmds.MoveTo_LeftMidAlliance.get(),
                        cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.mediumShoot.get());
        // Mini Autos

        // Depot
        Command DepotFaceIn = new SequentialCommandGroup(
                        cmds.MoveTo_leftOfDepotFaceIn.get(),
                        cmds.MoveTo_depotFaceIn.get(),
                        cmds.MoveTo_midOfDepotFaceIn.get(),
                        cmds.MoveTo_rightOfDepotFaceIn.get());
        // Bumps
        Command rightBumpToField = new SequentialCommandGroup(
                        // MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get());

        Command leftBumpToField = new SequentialCommandGroup(
                        cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_leftBump_AllianceToFieldEnd.get());

        Command rightBumpToAlliance = new SequentialCommandGroup(
                        // MoveTo_rightBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_rightBump_FieldToAllianceEnd.get());

        Command leftBumpToAlliance = new SequentialCommandGroup(
                        cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get());
        // Center Harvest
        Command CenterHarvest = new SequentialCommandGroup(
                        cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeStart.get(), cmds.intake.runIntakeOut()),
                        Commands.deadline(cmds.MoveTo_centerRightIntakeEnd.get(), cmds.intake.runPickupIn()),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeEndLookHub.get(), cmds.intake.runIntakeCenter()),
                        // MoveTo_centerRightIntakeEndLookHub.get(),
                        // intake.runIntakeCenter(),
                        cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                        cmds.MoveTo_allianceCenter.get(),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get());
        Command CenterRtoLSupplying = new SequentialCommandGroup(
                        cmds.MoveTo_leftLoadInZone.get(),
                        cmds.MoveTo_rightLoadInZone.get());

        // Actual Full autos
        Command LeftStartDepotScore = new SequentialCommandGroup(
                        cmds.MoveTo_leftOfDepotFaceIn.get(),
                        cmds.MoveTo_depotFaceIn.get(),
                        cmds.MoveTo_midOfDepotFaceIn.get(),
                        cmds.MoveTo_rightOfDepotFaceIn.get(),
                        cmds.MoveTo_FrontHubShoot.get());

        Command RightStartDepotScore = new SequentialCommandGroup(
                        cmds.MoveTo_allianceCenter.get(),
                        cmds.MoveTo_towerDodge_Start.get(),
                        cmds.MoveTo_towerDodge_End.get(),
                        cmds.MoveTo_rightOfDepotFaceOut.get(),
                        cmds.MoveTo_midOfDepotFaceOut.get(),
                        cmds.MoveTo_depotFaceOut.get(),
                        cmds.MoveTo_leftOfDepotFaceOut.get(),
                        cmds.MoveTo_FrontHubShoot.get());

        Command CenterStartFeedingStationScore = new SequentialCommandGroup(
                        // MoveTo_FrontHubShoot.get(),
                        // mediumShoot.get(),
                        cmds.MoveTo_feedingStation_centerStart.get(),
                        Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get());

        Command LeftStartFeedingStationScore = new SequentialCommandGroup(
                        // MoveTo_FrontHubShoot.get(),
                        // mediumShoot.get(),
                        cmds.MoveTo_feedingStation_leftStart.get(),
                        Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get());

        Command RightStartFeedingStationScore = new SequentialCommandGroup(
                        // MoveTo_FrontHubShoot.get(),
                        // mediumShoot.get(),
                        Commands.parallel(cmds.intake.runIntakeOut(), cmds.MoveTo_feedingStation.get()),
                        Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get());

        // TODO: Fix this
        Command RightStartCenterHarvestInLeft = new SequentialCommandGroup(
                        // MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeStart.get(), cmds.intake.runIntakeOut()),
                        Commands.deadline(cmds.MoveTo_centerRightIntakeEnd.get(), cmds.intake.runPickupIn()),
                        // MoveTo_centerRightIntakeStart.get(),
                        // MoveTo_centerRightIntakeEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeEndLookHub.get(),
                                        cmds.intake.runIntakeCenter()),
                        // MoveTo_centerRightIntakeEndLookHub.get(),
                        // intake.runIntakeCenter(),
                        cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                        // MoveTo_allianceCenter.get(),
                        // MoveTo_FrontHubShoot.get(),
                        cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.mediumShoot.get());

        Command LeftStartCenterHarvestInRight = new SequentialCommandGroup(
                        // MoveTo_leftBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerLeftIntakeStart.get(), cmds.intake.runIntakeOut()),
                        Commands.deadline(cmds.MoveTo_centerLeftIntakeEnd.get(), cmds.intake.runPickupIn()),
                        // MoveTo_centerLeftIntakeStart.get(),
                        // MoveTo_centerLeftIntakeEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerLeftIntakeEndLookHub.get(), cmds.intake.runIntakeCenter()),
                        // MoveTo_centerLeftIntakeEndLookHub.get(),
                        // intake.runIntakeCenter(),
                        cmds.MoveTo_rightBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                        // MoveTo_allianceCenter.get(),
                        // MoveTo_FrontHubShoot.get(),
                        cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.mediumShoot.get());

        Command RightQuad = new SequentialCommandGroup(
                        // MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeStart.get(), cmds.intake.runIntakeOut()),
                        Commands.deadline(
                                        Commands.sequence(
                                                        cmds.MoveTo_quadRight.get(),
                                                        cmds.MoveTo_rightQuadSecondSweep_Start.get()
                                        // MoveTo_rightQuadSecondSweep_End.get(),
                                        ), cmds.intake.runPickupIn()),
                        Commands.parallel(cmds.intake.runIntakeCenter(),
                                        cmds.MoveTo_rightBump_FieldToAllianceStart.get()),
                        cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                        cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.mediumShoot.get());

        Command LeftQuad = new SequentialCommandGroup(
                        // MoveTo_leftBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                        Commands.parallel(cmds.MoveTo_centerLeftIntakeStart.get(), cmds.intake.runIntakeOut()),
                        Commands.deadline(
                                        Commands.sequence(
                                                        cmds.MoveTo_quadLeft.get(),
                                                        cmds.MoveTo_leftQuadSecondSweep_Start.get()
                                        // MoveTo_leftQuadSecondSweep_End.get(),
                                        ), cmds.intake.runPickupIn()),

                        Commands.parallel(cmds.intake.runIntakeCenter(),
                                        cmds.MoveTo_leftBump_FieldToAllianceStart.get()),
                        // MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                        cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                        cmds.mediumShoot.get());

        // Actual name: RightFeedShootCenterHarvest
        Command Right_Yum_Middle = new SequentialCommandGroup(
                        cmds.MoveTo_feedingStation.get(),
                        Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.mediumShoot.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                        Commands.deadline(cmds.MoveTo_centerRightIntakeStart.get(),
                                        cmds.intake.runIntakeOut()),
                        Commands.deadline(cmds.MoveTo_centerRightIntakeEnd.get(), cmds.intake.runPickupIn()),
                        Commands.parallel(cmds.MoveTo_centerRightIntakeEndLookHub.get(), cmds.intake.runIntakeCenter()),
                        cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                        cmds.MoveTo_FrontHubShoot.get());

        Command LeftDepotShootCenterHarvestInLeftShoot = new SequentialCommandGroup(
                        cmds.MoveTo_leftOfDepotFaceIn.get(),
                        cmds.MoveTo_depotFaceIn.get(),
                        cmds.MoveTo_midOfDepotFaceIn.get(),
                        cmds.MoveTo_rightOfDepotFaceIn.get(),
                        cmds.MoveTo_FrontHubShoot.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                        cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                        cmds.MoveTo_centerRightIntakeStart.get(),
                        cmds.MoveTo_centerRightIntakeEnd.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                        cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                        cmds.MoveTo_allianceCenter.get(),
                        cmds.MoveTo_FrontHubShoot.get());

        Command fourMeters = new SequentialCommandGroup(
                        cmds.MoveTo_quadLeft.get());
        Command TheShowboater = new SequentialCommandGroup(
                        cmds.MoveTo_depot_BackFace_Start.get(),
                        cmds.MoveTo_depot_BackFace_End.get(),
                        cmds.MoveTo_midOfDepotFaceIn.get(),
                        cmds.MoveTo_rightOfDepotFaceIn.get(),
                        cmds.MoveTo_New_FrontHubShoot.get());

        /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                // Elastic
        // NamedCommands.registerCommand("blueCenter", blueCenter);
        public void registerAutos() {
                NamedCommands.registerCommand("FrontHubShoot", FrontHubShoot);
                NamedCommands.registerCommand("blueCenterToDepot", LeftStart_ToDepot);
                NamedCommands.registerCommand("TEST", TEST);
                NamedCommands.registerCommand("DepotFaceIn", DepotFaceIn);
                NamedCommands.registerCommand("JUSTSHOOT", JUSTSHOOT);
                NamedCommands.registerCommand("LeftStart_JUSTSHOOT", LeftStart_JUSTSHOOT);

                NamedCommands.registerCommand("rightBumpToField", rightBumpToField);
                NamedCommands.registerCommand("leftBumpToField", leftBumpToField);
                NamedCommands.registerCommand("rightBumpToAlliance", rightBumpToAlliance);
                NamedCommands.registerCommand("leftBumpToAlliance", leftBumpToAlliance);

                NamedCommands.registerCommand("CenterHarvest", CenterHarvest);
                NamedCommands.registerCommand("LeftStartDepotScore", LeftStartDepotScore);
                NamedCommands.registerCommand("RightStartDepotScore", RightStartDepotScore);

                // Feeding station
                NamedCommands.registerCommand("RightStartFeedingStationScore", RightStartFeedingStationScore);
                NamedCommands.registerCommand("LeftStartFeedingStationScore", LeftStartFeedingStationScore);

                NamedCommands.registerCommand("RightStartCenterHarvestInLeft", RightStartCenterHarvestInLeft);
                NamedCommands.registerCommand("LeftStartCenterHarvestInRight", LeftStartCenterHarvestInRight);
                NamedCommands.registerCommand("CenterStartFeedingStationScore",
                                CenterStartFeedingStationScore);
                NamedCommands.registerCommand("LeftDepotShootCenterHarvestInLeftShoot",
                                LeftDepotShootCenterHarvestInLeftShoot);
                NamedCommands.registerCommand("TheShowboater", TheShowboater);
                NamedCommands.registerCommand("RightQuad", RightQuad);
                NamedCommands.registerCommand("LeftQuad", LeftQuad);
                NamedCommands.registerCommand("fourMeters", fourMeters);

                // TODO: add window in Elastic
                OVERRIDE_AUTO_COMMAND = RightQuad;
        }
}
