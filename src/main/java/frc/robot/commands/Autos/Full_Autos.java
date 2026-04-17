// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                // Elastic
        // NamedCommands.registerCommand("blueCenter", blueCenter);
        public void registerAutos(CommandsForAutos cmds) {

                // #region TEST AUTOS (All caps)
                Command TEST = new SequentialCommandGroup(
                                // cmds.MoveTo_depot_BackFace_t.get(),
                                // cmds.intake.runIntakeOut(),
                                // cmds.MoveTo_depot_BackFace_In.get(),
                                // cmds.MoveTo_depot_BackFace_Out.get(),
                                cmds.MoveTo_New_FrontHubShoot.get());

                Command JUSTSHOOT = new SequentialCommandGroup(
                                // MoveTo_allianceCenter.get(),
                                // MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.MoveTo_FrontHubShoot.get(),
                                cmds.intake.runIntakeOut(),
                                // pickup.runPickupIn().withTimeout(0.5),
                                Commands.parallel(cmds.MoveTo_IntakeIN_FrontHubShoot.get(),
                                                cmds.pickup.runPickupIn().withTimeout(3.0)),
                                cmds.intake.runIntakeCenter(),
                                // MoveTo_depot_BackFace_In.get(),
                                cmds.MoveTo_IntakeOUT_FrontHubShoot.get());

                Command TESTCenterHarvest = new SequentialCommandGroup(
                                cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                cmds.MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB.get(),
                                cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                Commands.parallel(cmds.MoveTo_centerRightIntakeStart.get(), cmds.intake.runIntakeOut()),
                                Commands.deadline(cmds.MoveTo_centerRightIntakeEnd.get(), cmds.pickup.runPickupIn()),
                                Commands.parallel(cmds.MoveTo_centerRightIntakeEndLookHub.get(),
                                                cmds.intake.runIntakeCenter()),
                                // MoveTo_centerRightIntakeEndLookHub.get(),
                                // intake.runIntakeCenter(),
                                cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_allianceCenter.get(),
                                cmds.MoveTo_FrontHubShoot.get(),
                                cmds.mediumShoot.get());
                // #endregion

                Command JUST_SHOOT_FROM_ANYWHERE = Commands.sequence(cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("JUST_SHOOT_FROM_ANYWHERE");

                Command LeftStart_JUSTSHOOT = new SequentialCommandGroup(
                                cmds.MoveTo_LeftMidAlliance.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get());

                // // Depot
                // Command DepotFaceIn = new SequentialCommandGroup(
                // cmds.MoveTo_leftOfDepotFaceIn.get(),
                // cmds.MoveTo_depotFaceIn.get(),
                // cmds.MoveTo_midOfDepotFaceIn.get(),
                // cmds.MoveTo_rightOfDepotFaceIn.get());
                Command LeftStart_ToDepot = new SequentialCommandGroup(
                                // Commands.parallel(MoveTo_leftBump_AllianceToFieldStart.get(),
                                // intake.runIntakeOut()),
                                Commands.parallel(
                                                cmds.intake.runIntakeOut(),
                                                cmds.MoveTo_leftBump_AllianceToFieldStart.get()),
                                cmds.MoveTo_depot_BackFace_Out.get(),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_depot_BackFace_In.get(),
                                                                cmds.MoveTo_depot_BackFace_End_Dos.get(),
                                                                cmds.MoveTo_midOfDepotFaceOut.get()),
                                                cmds.pickup.runPickupIn()),

                                cmds.MoveTo_depot_BackFace_Out.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("LeftStart_ToDepot");
                // #region Bumps
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
                // #endregion
                // Center Harvest

                Command CenterRtoLSupplying = new SequentialCommandGroup(
                                cmds.MoveTo_leftLoadInZone.get(),
                                cmds.MoveTo_rightLoadInZone.get());

                // Actual Full autos
                // Command LeftStartDepotScore = new SequentialCommandGroup(
                // cmds.MoveTo_leftOfDepotFaceIn.get(),
                // cmds.MoveTo_depotFaceIn.get(),
                // cmds.MoveTo_midOfDepotFaceIn.get(),
                // cmds.MoveTo_rightOfDepotFaceIn.get(),
                // cmds.MoveTo_FrontHubShoot.get());

                // Command RightStartDepotScore = new SequentialCommandGroup(
                // cmds.MoveTo_allianceCenter.get(),
                // cmds.MoveTo_towerDodge_Start.get(),
                // cmds.MoveTo_towerDodge_End.get(),
                // cmds.MoveTo_rightOfDepotFaceOut.get(),
                // cmds.MoveTo_midOfDepotFaceOut.get(),
                // cmds.MoveTo_depotFaceOut.get(),
                // cmds.MoveTo_leftOfDepotFaceOut.get(),
                // cmds.MoveTo_FrontHubShoot.get());

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
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerRightIntakeStart.get(),
                                                                cmds.MoveTo_centerRightIntakeEnd.get(),
                                                                cmds.MoveTo_centerRightIntakeEndLookHub.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get())
                                .withName("RightStartCenterHarvestInLeft");

                Command LeftStartCenterHarvestInRight = new SequentialCommandGroup(
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerLeftIntakeStart.get(),
                                                                cmds.MoveTo_centerLeftIntakeEnd.get(),
                                                                cmds.MoveTo_centerLeftIntakeEndLookHub.get(),
                                                                cmds.MoveTo_rightBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("LeftStartCenterHarvestInRight");

                // Center Harvest Secondary Sweep

                Command RightStartCenterHarvest_SecondSweep_TOP_FIRST = new SequentialCommandGroup(
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerRightIntakeStart.get(),
                                                                cmds.MoveTo_centerRightIntakeEnd.get(),
                                                                cmds.MoveTo_centerRightIntake_SecondSweep.get(),
                                                                // cmds.MoveTo_centerLeftIntakeEndLookHub.get(),
                                                                cmds.MoveTo_rightBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("RightStartCenterHarvest_SecondSweep_TOP_FIRST");

                Command LeftStartCenterHarvest_SecondSweep_TOP_FIRST = new SequentialCommandGroup(
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerLeftIntakeStart.get(),
                                                                cmds.MoveTo_centerLeftIntakeEnd.get(),
                                                                // cmds.MoveTo_centerLeftIntakeEndLookHub.get(),
                                                                cmds.MoveTo_centerLeftIntake_SecondSweep.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("LeftStartCenterHarvest_SecondSweep_TOP_FIRST");
                // TODO:Check Rotations
                // TODO: Third Sweeps!!! (YAY!)
                Command RightStartCenterHarvest_SecondSweep_LOW_FIRST = new SequentialCommandGroup(
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerLeftIntake_SecondSweep.get(),
                                                                cmds.MoveTo_centerLeftIntakeStart.get(),
                                                                cmds.MoveTo_centerLeftIntakeEnd.get(),

                                                                // cmds.MoveTo_centerLeftIntakeEndLookHub.get(),
                                                                cmds.MoveTo_rightBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("RightStartCenterHarvest_SecondSweep");

                Command LeftStartCenterHarvest_SecondSweep_LOW_FIRST = new SequentialCommandGroup(
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerRightIntake_SecondSweep.get(),
                                                                cmds.MoveTo_centerRightIntakeStart.get(),
                                                                cmds.MoveTo_centerRightIntakeEnd.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),

                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("LeftStartCenterHarvest_SecondSweep");

                // Quads
                // TODO: Ask Stephen about curve
                // TODO: Is it high enough and is the angle good?
                Command RightQuad = new SequentialCommandGroup(
                                // Running start
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                // Cross bump
                                Commands.parallel(
                                                cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_quadRightIntakeStart.get(),
                                                                cmds.MoveTo_quadRight.get(),
                                                                cmds.MoveTo_centerLeftIntakeEnd.get(),
                                                                cmds.MoveTo_centerLeftIntakeEndLookHub.get()// ,
                                                // MoveTo_rightBump_FieldToAllianceStart.get() // move to the start of
                                                // the bump before crossing
                                                ),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(), // TODO run pickup while we move to this
                                                                                // position?
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("RightQuad");

                // TODO: Good enough?
                Command LeftQuad = new SequentialCommandGroup(
                                // Running start
                                Commands.deadline(cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                // Cross bump
                                Commands.parallel(
                                                cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_quadLeftIntakeStart.get(),
                                                                cmds.MoveTo_quadLeft.get(),
                                                                cmds.MoveTo_centerRightIntakeEnd.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.mediumShoot.get()).withName("LeftQuad");

                Command Zac_RightQuad = new SequentialCommandGroup(
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.parallel(cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerRightIntakeStart.get(),
                                                                cmds.MoveTo_quadRight.get(),
                                                                cmds.MoveTo_rightQuadSecondSweep_Start.get(),
                                                                cmds.MoveTo_rightQuadSecondSweep_End.get(),
                                                                cmds.MoveTo_rightBump_FieldToAllianceStart.get()),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("Zac_RightQuad");

                Command Zac_LeftQuad = new SequentialCommandGroup(
                                cmds.MoveTo_leftBump_AllianceToFieldStart.get(), // TODO add deadline run deploy intake
                                                                                 // out
                                Commands.parallel(cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerLeftIntakeStart.get(),
                                                                cmds.MoveTo_quadLeft.get(),
                                                                cmds.MoveTo_leftQuadSecondSweep_Start.get(),
                                                                // MoveTo_leftQuadSecondSweep_End.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("Zac_LeftQuad");

                Command Leighton_RightQuad = new SequentialCommandGroup(
                                // Running start
                                Commands.deadline(cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                // Cross bump
                                Commands.parallel(
                                                cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerRightIntakeStart.get(),
                                                                cmds.MoveTo_quadRight.get(),
                                                                cmds.MoveTo_centerLeftIntakeEndLookHub.get()// ,
                                                // MoveTo_rightBump_FieldToAllianceStart.get() // move to the start of
                                                // the bump before crossing
                                                ),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_rightBump_FieldToAllianceEnd.get(), // TODO run pickup while we move to this
                                                                                // position?
                                cmds.MoveTo_ClosestShootingPosition_MEDIUM.get(),
                                cmds.mediumShoot.get()).withName("RightQuad");

                Command Leighton_LeftQuad = new SequentialCommandGroup(
                                // Running start
                                Commands.deadline(cmds.MoveTo_leftBump_AllianceToFieldStart.get(),
                                                cmds.intake.runIntakeOut()),
                                // Cross bump
                                Commands.parallel(
                                                cmds.MoveTo_leftBump_AllianceToFieldEnd.get(),
                                                cmds.intake.runIntakeOut()),
                                Commands.deadline(
                                                Commands.sequence(
                                                                cmds.MoveTo_centerLeftIntakeStart.get(),
                                                                cmds.MoveTo_quadLeft.get(),
                                                                cmds.MoveTo_leftBump_FieldToAllianceStart.get()),
                                                cmds.pickup.runPickupIn()),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.mediumShoot.get()).withName("LeftQuad");
                // TODO: Leeland's ideas
                Command RIGHT_THIS_WAS_LEELANDS_IDEA = new SequentialCommandGroup(

                );
                Command LEFT_THIS_WAS_LEELANDS_IDEA = new SequentialCommandGroup(

                );
                // #region Useless Autos
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
                                Commands.deadline(cmds.MoveTo_centerRightIntakeEnd.get(), cmds.pickup.runPickupIn()),
                                Commands.parallel(cmds.MoveTo_centerRightIntakeEndLookHub.get(),
                                                cmds.intake.runIntakeCenter()),
                                cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_FrontHubShoot.get());

                Command LeftDepotShootCenterHarvestInLeftShoot = new SequentialCommandGroup(
                                // cmds.MoveTo_leftOfDepotFaceIn.get(),
                                // cmds.MoveTo_depotFaceIn.get(),
                                // cmds.MoveTo_midOfDepotFaceIn.get(),
                                // cmds.MoveTo_rightOfDepotFaceIn.get(),
                                cmds.MoveTo_FrontHubShoot.get(),
                                cmds.MoveTo_rightBump_AllianceToFieldStart.get(),
                                cmds.MoveTo_rightBump_AllianceToFieldEnd.get(),
                                cmds.MoveTo_centerRightIntakeStart.get(),
                                cmds.MoveTo_centerRightIntakeEnd.get(),
                                cmds.MoveTo_leftBump_FieldToAllianceStart.get(),
                                cmds.MoveTo_leftBump_FieldToAllianceEnd.get(),
                                cmds.MoveTo_allianceCenter.get(),
                                cmds.MoveTo_FrontHubShoot.get());
                // #endregion
                Command fourMeters = new SequentialCommandGroup(
                                cmds.MoveTo_quadLeft.get()).withName("fourMeters");
                Command TheShowboater = new SequentialCommandGroup(
                                cmds.MoveTo_leftOfDepot_Out.get(),
                                cmds.MoveTo_leftOfDepot_In.get(),
                                cmds.MoveTo_leftOfDepot_Out.get(),
                                cmds.MoveTo_left_midOfDepot_Out.get(),
                                cmds.MoveTo_left_midOfDepot_In.get(),
                                cmds.MoveTo_left_midOfDepot_Out.get(),
                                cmds.MoveTo_depot_BackFace_Out.get(),
                                cmds.MoveTo_depot_BackFace_In.get(),
                                cmds.MoveTo_depot_BackFace_Out.get(),
                                cmds.MoveTo_rightOfDepot_Out.get(),
                                cmds.MoveTo_rightOfDepot_In.get(),
                                cmds.MoveTo_rightOfDepot_Out.get(),
                                cmds.MoveTo_ClosestShootingPosition_LONG.get(),
                                cmds.mediumShoot.get()).withName("TheShowboater");

                NamedCommands.registerCommand("LeftStart_ToDepot", LeftStart_ToDepot);

                // NamedCommands.registerCommand("rightBumpToField", rightBumpToField);
                // NamedCommands.registerCommand("leftBumpToField", leftBumpToField);
                // NamedCommands.registerCommand("rightBumpToAlliance", rightBumpToAlliance);
                // NamedCommands.registerCommand("leftBumpToAlliance", leftBumpToAlliance);

                NamedCommands.registerCommand("JUST_SHOOT_FROM_ANYWHERE", JUST_SHOOT_FROM_ANYWHERE);
                NamedCommands.registerCommand("RightStartCenterHarvestInLeft", RightStartCenterHarvestInLeft);
                NamedCommands.registerCommand("LeftStartCenterHarvestInRight", LeftStartCenterHarvestInRight);
                NamedCommands.registerCommand("RightStartCenterHarvest_SecondSweep_TOP_FIRST",
                                RightStartCenterHarvest_SecondSweep_TOP_FIRST);
                NamedCommands.registerCommand("LeftStartCenterHarvest_SecondSweep_TOP_FIRST",
                                LeftStartCenterHarvest_SecondSweep_TOP_FIRST);
                NamedCommands.registerCommand("RightStartCenterHarvest_SecondSweep_LOW_FIRST",
                                RightStartCenterHarvest_SecondSweep_LOW_FIRST);
                NamedCommands.registerCommand("LeftStartCenterHarvest_SecondSweep_LOW_FIRST",
                                LeftStartCenterHarvest_SecondSweep_LOW_FIRST);

                NamedCommands.registerCommand("RightQuad", RightQuad);
                NamedCommands.registerCommand("LeftQuad", LeftQuad);
                NamedCommands.registerCommand("Leighton_RightQuad", Leighton_RightQuad);
                NamedCommands.registerCommand("Leighton_LeftQuad", Leighton_LeftQuad);
                NamedCommands.registerCommand("Zac_RightQuad", Zac_RightQuad);
                NamedCommands.registerCommand("Zac_LeftQuad", Zac_LeftQuad);

                // NamedCommands.registerCommand("LeftStartDepotScore", LeftStartDepotScore);
                // NamedCommands.registerCommand("RightStartDepotScore", RightStartDepotScore);

                // Feeding station
                NamedCommands.registerCommand("RightStartFeedingStationScore", RightStartFeedingStationScore);
                NamedCommands.registerCommand("LeftStartFeedingStationScore", LeftStartFeedingStationScore);

                NamedCommands.registerCommand("CenterStartFeedingStationScore",
                                CenterStartFeedingStationScore);
                NamedCommands.registerCommand("LeftDepotShootCenterHarvestInLeftShoot",
                                LeftDepotShootCenterHarvestInLeftShoot);
                NamedCommands.registerCommand("TheShowboater", TheShowboater);
                NamedCommands.registerCommand("TEST", TEST);
                // TODO: use DepotFaceIn
                // NamedCommands.registerCommand("DepotFaceIn", DepotFaceIn);
                NamedCommands.registerCommand("JUSTSHOOT", JUSTSHOOT);
                NamedCommands.registerCommand("LeftStart_JUSTSHOOT", LeftStart_JUSTSHOOT);
                NamedCommands.registerCommand("TESTCenterHarvest", TESTCenterHarvest);
                NamedCommands.registerCommand("fourMeters", fourMeters);

                // TODO: Actually cook in autos
                OVERRIDE_AUTO_COMMAND = JUST_SHOOT_FROM_ANYWHERE;

                SmartDashboard.putString("Auto/SELECTED OVERRIDE_AUTO_COMMAND",
                                OVERRIDE_AUTO_COMMAND.getName());
        }
}
