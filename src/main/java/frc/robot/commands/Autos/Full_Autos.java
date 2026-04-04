// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoPilot.CommandsForAutos;

import static frc.robot.commands.AutoPilot.CommandsForAutos.*;
/** Add your docs here. */
public class Full_Autos {
    /* Full Autos */ // TODO: DON'T FORGET THE COMMAS
            public CommandsForAutos commandsForAutos;
            public Full_Autos(CommandsForAutos commandsForAutos){
                    this.commandsForAutos = commandsForAutos;
            }
                Command FrontHubShoot = new SequentialCommandGroup(
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());
                Command LeftStart_ToDepot = new SequentialCommandGroup(
                                // Commands.parallel(MoveTo_leftBump_AllianceToFieldStart.get(),
                                // intake.runIntakeOut()),

                                Commands.parallel(intake.runIntakeOut(),
                                                MoveTo_leftBump_AllianceToFieldStart.get()),
                                MoveTo_depot_BackFace_Start.get(),
                                Commands.deadline(
                                                Commands.sequence(MoveTo_depot_BackFace_End.get(),
                                                                MoveTo_depot_BackFace_End_Dos.get(),
                                                                MoveTo_midOfDepotFaceOut.get()),
                                                intake.runPickupIn()),

                                MoveTo_depot_BackFace_Start.get(),

                                // intake.runPickupStop(),
                                MoveTo_New_FrontHubShoot.get());

                Command TEST = new SequentialCommandGroup(
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_IntakeIN_FrontHubShoot.get(),
                                // shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT),
                                // new SetHoodPosition(hood, HoodAngles.SHORT),
                                // new AutoFire(shooter, indexer, hopper, () ->
                                // ShooterPreferences.INDEXER_VELOCITY)

                                // Commands.waitSeconds(0.5),
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_IntakeOUT_FrontHubShoot.get(),
                                // intake.runIntakeOut()
                                // TODO: Add intaking stuff
                                MoveTo_depot_BackFace_Start.get(),
                                intake.runIntakeOut(),
                                MoveTo_depot_BackFace_End.get(),

                                MoveTo_depot_BackFace_Start.get(),
                                MoveTo_New_FrontHubShoot.get());
                Command JUSTSHOOT = new SequentialCommandGroup(
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                MoveTo_FrontHubShoot.get(),
                                intake.runIntakeOut(),
                                // intake.runPickupIn().withTimeout(0.5),
                                Commands.parallel(MoveTo_IntakeIN_FrontHubShoot.get(),
                                                intake.runPickupIn().withTimeout(3.0)),
                                intake.runIntakeCenter(),
                                // MoveTo_depot_BackFace_End.get(),
                                MoveTo_IntakeOUT_FrontHubShoot.get()

                // MoveTo_rightBump_AllianceToFieldStart.get(),
                // MoveTo_rightBump_AllianceToFieldEnd.get(),
                // MoveTo_centerRightIntakeStart.get(),
                // MoveTo_centerRightIntakeEnd.get(),
                // MoveTo_leftLoadInZone.get()
                // MoveTo_rightLoadInZone.get()
                );
                Command LeftStart_JUSTSHOOT = new SequentialCommandGroup(
                                MoveTo_LeftMidAlliance.get(),
                                MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                mediumShoot.get());
                // Mini Autos

                // Depot
                Command DepotFaceIn = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get());
                // Bumps
                Command rightBumpToField = new SequentialCommandGroup(
                                // MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get());

                Command leftBumpToField = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get());

                Command rightBumpToAlliance = new SequentialCommandGroup(
                                // MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get());

                Command leftBumpToAlliance = new SequentialCommandGroup(
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get());
                // Center Harvest
                Command CenterHarvest = new SequentialCommandGroup(
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.parallel(MoveTo_centerRightIntakeStart.get(), intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn()),
                                Commands.parallel(MoveTo_centerRightIntakeEndLookHub.get(), intake.runIntakeCenter()),
                                // MoveTo_centerRightIntakeEndLookHub.get(),
                                intake.runIntakeCenter(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());
                Command CenterRtoLSupplying = new SequentialCommandGroup(
                                MoveTo_leftLoadInZone.get(),
                                MoveTo_rightLoadInZone.get());

                // Actual Full autos
                Command LeftStartDepotScore = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get());

                Command RightStartDepotScore = new SequentialCommandGroup(
                                MoveTo_allianceCenter.get(),
                                MoveTo_towerDodge_Start.get(),
                                MoveTo_towerDodge_End.get(),
                                MoveTo_rightOfDepotFaceOut.get(),
                                MoveTo_midOfDepotFaceOut.get(),
                                MoveTo_depotFaceOut.get(),
                                MoveTo_leftOfDepotFaceOut.get(),
                                MoveTo_FrontHubShoot.get());

                Command CenterStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                MoveTo_feedingStation_centerStart.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command LeftStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                MoveTo_feedingStation_leftStart.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command RightStartFeedingStationScore = new SequentialCommandGroup(
                                // MoveTo_FrontHubShoot.get(),
                                // mediumShoot.get(),
                                Commands.parallel(intake.runIntakeOut(), MoveTo_feedingStation.get()),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_FEEDER_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                // TODO: Fix this
                Command RightStartCenterHarvestInLeft = new SequentialCommandGroup(
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(),
                                                intake.runIntakeOut()),
                                // Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn());
                                MoveTo_centerRightIntakeStart.get(),
                                MoveTo_centerRightIntakeEnd.get(),
                                // Commands.parallel(MoveTo_centerRightIntakeEndLookHub.get(),
                                // intake.runIntakeCenter());
                                MoveTo_centerRightIntakeEndLookHub.get(),
                                // intake.runIntakeCenter(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command LeftStartCenterHarvestInRight = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldStart_LOOK_HUB.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get(),
                                // MoveAndDeploy_centerLeftIntakeStart.get(),
                                MoveTo_centerLeftIntakeStart.get(),
                                MoveTo_centerLeftIntakeEnd.get(),
                                // MoveToPickup_centerLeftIntakeEnd.get(),
                                // MoveToIntakeUp_centerLeftIntakeEndLookHub.get(),
                                MoveTo_centerLeftIntakeEndLookHub.get(),
                                // intake.runIntakeCenter(),
                                MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                Command RightQuad = new SequentialCommandGroup(
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(),
                                                intake.runIntakeOut()),
                                MoveTo_centerRightIntakeStart.get(),
                                MoveToPickup_quadLeft.get(),
                                MoveToIntakeUp_quadLeft.get(),
                                // MoveTo_quadLeft.get(),
                                MoveTo_rightBump_FieldToAllianceStart.get(),
                                MoveTo_rightBump_FieldToAllianceEnd.get());

                Command LeftQuad = new SequentialCommandGroup(
                                MoveTo_leftBump_AllianceToFieldStart.get(),
                                MoveTo_leftBump_AllianceToFieldEnd.get(),
                                MoveAndDeploy_centerLeftIntakeStart.get(),
                                MoveToPickup_quadRight.get(),
                                MoveToIntakeUp_quadRight.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get());

                // Actual name: RightFeedShootCenterHarvest
                Command Right_Yum_Middle = new SequentialCommandGroup(
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_FrontHubShoot.get(),
                                mediumShoot.get(),
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.deadline(MoveTo_centerRightIntakeStart.get(),
                                                intake.runIntakeOut()),
                                Commands.deadline(MoveTo_centerRightIntakeEnd.get(), intake.runPickupIn()),
                                Commands.parallel(MoveTo_centerRightIntakeEndLookHub.get(), intake.runIntakeCenter()),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_FrontHubShoot.get());

                Command LeftDepotShootCenterHarvestInLeftShoot = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_FrontHubShoot.get(),
                                MoveTo_rightBump_AllianceToFieldStart.get(),
                                MoveTo_rightBump_AllianceToFieldEnd.get(),
                                MoveTo_centerRightIntakeStart.get(),
                                MoveTo_centerRightIntakeEnd.get(),
                                MoveTo_leftBump_FieldToAllianceStart.get(),
                                MoveTo_leftBump_FieldToAllianceEnd.get(),
                                MoveTo_allianceCenter.get(),
                                MoveTo_FrontHubShoot.get());

                Command fourMeters = new SequentialCommandGroup(
                                intake.runIntakeOut(),
                                Commands.parallel(MoveTo_fourMeters.get(), intake.runPickupIn()),
                                intake.runPickupStop());
                Command TheShowboater = new SequentialCommandGroup(
                                MoveTo_depot_BackFace_Start.get(),
                                MoveTo_depot_BackFace_End.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_New_FrontHubShoot.get());

                /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                        // Elastic
                // NamedCommands.registerCommand("blueCenter", blueCenter);
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
