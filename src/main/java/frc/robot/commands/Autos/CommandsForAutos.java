package frc.robot.commands.Autos;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.therekrab.autopilot.APConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.commands.DriveToHubDistance;
import frc.robot.commands.AutoPilot.AutoPilotV2Command;
import frc.robot.commands.AutoPilot.AutoPilotV2Command.Builder;
import frc.robot.commands.Shooter.AutoFire;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.AutoCommands;
import frc.robot.subsystems.AdjustableHood;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_XY_THRESHOLD;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_THETA_THRESHOLD;
import static frc.robot.commands.AutoPilot.AutoPilotV2Command.DEFAULT_BEELINE_THRESHOLD;
import static frc.robot.commands.Autos.AutoPoses.*;

public class CommandsForAutos {
        // TODO: integrate shooting positions dropdown into all autos, include the shoot
        // from distance in the dropdown
        // TODO: add a start position dropdown, integrate into all autos
        // TODO: integrate shooting positions dropdown into all autos, include the shoot
        // from distance in the dropdown
        // TODO: add a start position dropdown, integrate into all autos

        // Velocity is max speed overall/ in a sec, how much can position change
        // Acceleration in a second, how much can velocity change
        // Jerk is how fast it starts and stops

        // #region CONSTRAINTS
        private static final APConstraints bumpConstraints = new APConstraints()
                        .withAcceleration(30) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(6)
                        .withJerk(600);

        private static final double BUMP_NARROW_XY_THRESHOLD_CM = 12;
        private static final double BUMP_WIDE_XY_THRESHOLD_CM = 30;
        private static final double BUMP_THETA_THRESHOLD_DEG = 12;

        // TODO: Adjust
        private static final APConstraints fullFieldConstraints = new APConstraints()
                        .withAcceleration(1.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(4.0)
                        .withJerk(67);

        private static final TrapezoidProfile.Constraints centerHarvestConstraint = new TrapezoidProfile.Constraints(
                        0.5, 0.5);
        // .withJerk(67.0);
        // #endregion
        CommandSwerveDrivetrain drivetrain;
        Climber climber;
        Intake intake;
        Hopper hopper;
        Indexer indexer;
        Shooter shooter;
        AdjustableHood hood;

        public CommandsForAutos(CommandSwerveDrivetrain drivetrain, Climber climber,
                        Intake intake,
                        Hopper hopper,
                        Indexer indexer,
                        Shooter shooter,
                        AdjustableHood hood) {

                this.drivetrain = drivetrain;
                this.climber = climber;
                this.intake = intake;
                this.hopper = hopper;
                this.indexer = indexer;
                this.shooter = shooter;
                this.hood = hood;

        }

        /* Commands */
        // Uses command suppliers instead of commands so that we can reuse the same
        // command in an autonomous
        Supplier<Command> MoveTo_allianceCenter = () -> new AutoPilotV2Command.Builder(
                        () -> FieldConstants.BLUE_HUB_SHOOT_CLOSE, drivetrain, "MoveTo_allianceCenter")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_LeftMidAlliance = () -> new AutoPilotV2Command.Builder(
                        () -> LeftMidAlliance.get(), drivetrain, "MoveTo_LeftMidAlliance")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_centerOfField = () -> new AutoPilotV2Command.Builder(
                        () -> centerOfField.get(), drivetrain, "MoveTo_centerOfField")
                        .withFlipPoseForAlliance(true)
                        .build();

        // #region Feeding station Movements
        Supplier<Command> MoveTo_feedingStation = () -> new AutoPilotV2Command.Builder(
                        () -> feedingStation.get(), drivetrain, "MoveTo_feedingStation")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_feedingStation_leftStart = () -> Commands.parallel(intake.runIntakeOut(),
                        Commands.sequence(MoveTo_allianceCenter.get(),
                                        MoveTo_feedingStation.get()));
        Supplier<Command> MoveTo_feedingStation_centerStart = () -> Commands.parallel(intake.runIntakeOut(),
                        Commands.sequence(
                                        MoveTo_allianceCenter.get(),
                                        MoveTo_feedingStation.get()));

        Supplier<Command> MoveTo_fourMeters = () -> new AutoPilotV2Command.Builder(
                        () -> fourMeters.get(), drivetrain, "MoveTo_fourMeters")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(fullFieldConstraints)
                        .build();
        // #endregion

        // Shooter
        Supplier<Command> MoveTo_FrontHubShoot = () -> new DriveToHubDistance(drivetrain,
                        FieldConstants.ALLIANCE_HUB_POSITION,
                        shooter.getDistanceFromSpeed());
        Supplier<Command> MoveTo_New_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                        () -> FrontHubShoot.get(), drivetrain, "MoveTo_New_FrontHubShoot")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_requestedSpeedDistanceToHub = () -> new DriveToHubDistance(drivetrain,
                        FieldConstants.ALLIANCE_HUB_POSITION,
                        shooter.getDistanceFromSpeed());

        // Move To Shooting Positions
        Supplier<Command> MoveTo_ClosestShootingPosition_SHORT = () -> Commands.sequence(
                        new InstantCommand(() -> shooter
                                        .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.SHORT)),
                        MoveTo_requestedSpeedDistanceToHub.get());
        Supplier<Command> MoveTo_ClosestShootingPosition_MEDIUM = () -> Commands.sequence(
                        new InstantCommand(() -> shooter
                                        .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.MEDIUM)),
                        MoveTo_requestedSpeedDistanceToHub.get());
        Supplier<Command> MoveTo_ClosestShootingPosition_LONG = () -> Commands.sequence(
                        new InstantCommand(() -> shooter
                                        .setRequestedSpeedWithoutShooting(() -> ShooterPreferences.LONG)),
                        MoveTo_requestedSpeedDistanceToHub.get());

        Supplier<Command> MoveTo_IntakeIN_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                        () -> IntakeIN_FrontHubShoot.get(), drivetrain, "MoveTo_IntakeIN_FrontHubShoot")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_IntakeOUT_FrontHubShoot = () -> new AutoPilotV2Command.Builder(
                        () -> IntakeOUT_FrontHubShoot.get(), drivetrain, "MoveTo_IntakeOUT_FrontHubShoot")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_RightHubShoot = () -> new AutoPilotV2Command.Builder(
                        () -> RightHubShoot.get(), drivetrain, "MoveTo_RightHubShoot")
                        .withFlipPoseForAlliance(true)
                        .build();
        // Center Harvest(s)
        Supplier<Command> MoveTo_centerRightIntakeStart = () -> new AutoPilotV2Command.Builder(
                        () -> centerRightIntakeStart.get(), drivetrain, "MoveTo_centerRightIntakeStart")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_centerRightIntakeEnd = () -> new AutoPilotV2Command.Builder(
                        () -> centerRightIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                        .withFlipPoseForAlliance(true)
                        // TODO: .withConstraints(fullFieldConstraints)
                        .build();

        Supplier<Command> MoveTo_centerLeftIntakeStart = () -> new AutoPilotV2Command.Builder(
                        () -> centerLeftIntakeStart.get(), drivetrain, "MoveTo_centerLeftIntakeStart")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_centerLeftIntakeEnd = () -> new AutoPilotV2Command.Builder(
                        () -> centerLeftIntakeEnd.get(), drivetrain, "MoveTo_centerRightIntakeEnd")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(centerHarvestConstraint)
                        .build();
        Supplier<Command> MoveTo_rightQuadSecondSweep_Start = () -> new AutoPilotV2Command.Builder(
                        () -> rightQuadSecondSweep_Start.get(), drivetrain, "MoveTo_rightQuadSecondSweep_Start")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_rightQuadSecondSweep_End = () -> new AutoPilotV2Command.Builder(
                        () -> rightQuadSecondSweep_End.get(), drivetrain, "MoveTo_rightQuadSecondSweep_End")
                        .withFlipPoseForAlliance(true)
                        // TODO: .withConstraints(fullFieldConstraints)
                        .build();

        Supplier<Command> MoveTo_leftQuadSecondSweep_Start = () -> new AutoPilotV2Command.Builder(
                        () -> leftQuadSecondSweep_Start.get(), drivetrain, "MoveTo_leftQuadSecondSweep_Start")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_leftQuadSecondSweep_End = () -> new AutoPilotV2Command.Builder(
                        () -> leftQuadSecondSweep_End.get(), drivetrain, "MoveTo_leftQuadSecondSweep_End")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(centerHarvestConstraint)
                        .build();

        // deadline runs in
        // parrallel until the
        // first command
        // finishes
        Supplier<Command> MoveTo_quadRight = () -> new AutoPilotV2Command.Builder(
                        () -> quadRight.get(), drivetrain, "MoveTo_quadRight")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_quadLeft = () -> new AutoPilotV2Command.Builder(
                        () -> quadLeft.get(), drivetrain, "MoveTo_quadLeft")
                        .withFlipPoseForAlliance(true)
                        .build();

        Supplier<Command> MoveTo_centerRightIntakeEndLookHub = () -> new AutoPilotV2Command.Builder(
                        () -> centerRightIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                        .withFlipPoseForAlliance(true)
                        .withProfileThresholds(
                                        16.0, 5.0, DEFAULT_BEELINE_THRESHOLD)
                        // .withWaitSeconds(1)
                        .build();

        Supplier<Command> MoveTo_centerLeftIntakeEndLookHub = () -> new AutoPilotV2Command.Builder(
                        () -> centerLeftIntakeEndLookHub.get(), drivetrain, "centerRightIntakeEndLookHub")
                        .withFlipPoseForAlliance(true)
                        // .withWaitSeconds(1)
                        .build();

        Supplier<Command> MoveTo_rightLoadInZone = () -> new AutoPilotV2Command.Builder(
                        () -> rightLoadInZone.get(), drivetrain, "MoveTo_rightLoadInZone")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_leftLoadInZone = () -> new AutoPilotV2Command.Builder(
                        () -> leftLoadInZone.get(), drivetrain, "MoveTo_leftLoadInZone")
                        .withFlipPoseForAlliance(true)
                        .build();
        // Depot
        Supplier<Command> MoveTo_depotFaceIn = () -> new AutoPilotV2Command.Builder(
                        () -> depotFaceIn.get(), drivetrain, "MoveTo_depotFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_leftOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                        () -> leftOfDepotFaceIn.get(), drivetrain, "MoveTo_leftOfDepotFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_midOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                        () -> midOfDepotFaceIn.get(), drivetrain, "MoveTo_midOfDepotFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_rightOfDepotFaceIn = () -> new AutoPilotV2Command.Builder(
                        () -> rightOfDepotFaceIn.get(), drivetrain, "MoveTo_rightOfDepotFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_depotFaceOut = () -> new AutoPilotV2Command.Builder(
                        () -> depotFaceOut.get(), drivetrain, "MoveTo_depotFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_leftOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                        () -> leftOfDepotFaceOut.get(), drivetrain, "MoveTo_leftOfDepotFaceOut")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_midOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                        () -> midOfDepotFaceOut.get(), drivetrain, "MoveTo_midOfDepotFaceOut")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_rightOfDepotFaceOut = () -> new AutoPilotV2Command.Builder(
                        () -> rightOfDepotFaceOut.get(), drivetrain, "MoveTo_rightOfDepotFaceOut")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_depot_BackFace_Start = () -> new AutoPilotV2Command.Builder(
                        () -> depot_BackFace_Start.get(), drivetrain, "MoveTo_depot_BackFace_Start")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_depot_BackFace_End = () -> new AutoPilotV2Command.Builder(
                        () -> depot_BackFace_End.get(), drivetrain, "MoveTo_depot_BackFace_End")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_depot_BackFace_End_Dos = () -> new AutoPilotV2Command.Builder(
                        () -> depot_BackFace_End_Dos.get(), drivetrain, "MoveTo_depot_BackFace_End_Dos")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_towerDodge_Start = () -> new AutoPilotV2Command.Builder(
                        () -> towerDodge_Start.get(), drivetrain, "MoveTo_towerDodge_Start")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_towerDodge_End = () -> new AutoPilotV2Command.Builder(
                        () -> towerDodge_End.get(), drivetrain, "MoveTo_towerDodge_End")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_selectedShootPosition = () -> new AutoPilotV2Command.Builder(
                        AutoCommands.getShootPosition(), drivetrain, "MoveTo_selectedShootPosition")
                        .withFlipPoseForAlliance(true)
                        .build();
        // right from the driver station view
        // Bump Stuff

        Supplier<Command> MoveTo_leftBump_AllianceToFieldStart = () -> new AutoPilotV2Command.Builder(
                        () -> leftBump_AllianceToFieldStart.get(), drivetrain,
                        "MoveTo_leftBump_AllianceToFieldStart")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(bumpConstraints)
                        .build();
        Supplier<Command> MoveTo_leftBump_AllianceToFieldStart_LOOK_HUB = () -> new AutoPilotV2Command.Builder(
                        () -> leftBump_AllianceToFieldStart_LOOK_HUB.get(), drivetrain,
                        "MoveTo_leftBump_AllianceToFieldStart_LOOK_HUB")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(bumpConstraints)
                        .build();
        Supplier<Command> MoveTo_leftBump_AllianceToFieldEnd = () -> new AutoPilotV2Command.Builder(
                        () -> leftBump_AllianceToFieldEnd.get(), drivetrain,
                        "MoveTo_leftBump_AllianceToFieldEnd")
                        .withFlipPoseForAlliance(true)
                        .withConstraints(bumpConstraints)
                        .build();

        Supplier<Command> MoveTo_rightBump_AllianceToFieldStart = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_AllianceToFieldStart.get(), drivetrain,
                        "MoveTo_rightBump_AllianceToFieldStart")
                        .withFlipPoseForAlliance(true)
                        .withProfileThresholds(BUMP_NARROW_XY_THRESHOLD_CM, BUMP_THETA_THRESHOLD_DEG,
                                        DEFAULT_BEELINE_THRESHOLD)
                        // .withConstraints(bumpConstraints)
                        .build();
        Supplier<Command> MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_AllianceToFieldStart_LOOK_HUB.get(), drivetrain,
                        "MoveTo_rightBump_AllianceToFieldStart_LOOK_HUB")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(bumpConstraints)
                        .build();

        // Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new
        // AutoPilotV2Command.Builder(
        // () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
        // "MoveTo_rightBump_AllianceToFieldEnd")
        // .withFlipPoseForAlliance(true)
        // .build();

        Supplier<Command> MoveTo_rightBump_AllianceToFieldEnd = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_AllianceToFieldEnd.get(), drivetrain,
                        "MoveTo_rightBump_AllianceToFieldEnd")
                        .withFlipPoseForAlliance(true)
                        .withConstraints(bumpConstraints)
                        .withProfileThresholds(
                                        BUMP_WIDE_XY_THRESHOLD_CM, BUMP_THETA_THRESHOLD_DEG,
                                        DEFAULT_BEELINE_THRESHOLD)
                        .build();

        Supplier<Command> MoveTo_leftBump_FieldToAllianceStart = () -> new AutoPilotV2Command.Builder(
                        () -> leftBump_FieldToAllianceStart.get(), drivetrain,
                        "MoveTo_leftBump_FieldToAllianceStart")
                        .withFlipPoseForAlliance(true)
                        // .withConstraints(bumpConstraints)
                        .build();

        Supplier<Command> MoveTo_leftBump_FieldToAllianceEnd = () -> new AutoPilotV2Command.Builder(
                        () -> leftBump_FieldToAllianceEnd.get(), drivetrain,
                        "MoveTo_leftBump_FieldToAllianceEnd")
                        .withFlipPoseForAlliance(true)
                        .withProfileThresholds(6, 12, DEFAULT_BEELINE_THRESHOLD)
                        .withConstraints(bumpConstraints)
                        .build();
        Supplier<Command> MoveTo_rightBump_FieldToAllianceStart = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_FieldToAllianceStart.get(), drivetrain,
                        "MoveTo_rightBump_FieldToAllianceStart")
                        .withFlipPoseForAlliance(true)
                        .withProfileThresholds(BUMP_NARROW_XY_THRESHOLD_CM, BUMP_THETA_THRESHOLD_DEG,
                                        DEFAULT_BEELINE_THRESHOLD)
                        .build();
        Supplier<Command> MoveTo_rightBump_FieldToAllianceEnd = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_FieldToAllianceEnd.get(), drivetrain,
                        "MoveTo_rightBump_FieldToAllianceEnd")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_rightBump_FieldToAllianceEndDos = () -> new AutoPilotV2Command.Builder(
                        () -> rightBump_FieldToAllianceEndDos.get(), drivetrain,
                        "MoveTo_rightBump_FieldToAllianceEndDos")
                        .withFlipPoseForAlliance(true)
                        .build();
        Supplier<Command> MoveTo_startRightFaceIn = () -> new AutoPilotV2Command.Builder(
                        () -> startRightFaceIn.get(), drivetrain,
                        "MoveTo_startRightFaceIn")
                        .withFlipPoseForAlliance(true)
                        .build();
        // Supplier<Command> quickShoot = () -> Commands.sequence(
        // shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG),
        // new AutoFire(shooter, indexer, hopper, () ->
        // ShooterPreferences.INDEXER_VELOCITY)
        // .repeatedly())
        // .withTimeout(3)
        // .andThen(Commands.sequence(
        // indexer.runStopIndexer(),
        // shooter.stopShooter(),
        // indexer.runStopIndexer()));

        Supplier<Command> mediumShoot = () -> Commands.sequence(
                        shooter.runSetRequestedSpeed(() -> ShooterPreferences.MEDIUM),
                        AutoFire.autonomous(shooter, indexer,
                                        () -> ShooterPreferences.INDEXER_VELOCITY))
                        .withTimeout(10)
                        .andThen(Commands.sequence(
                                        indexer.runStopIndexer(),
                                        shooter.stopShooter(),
                                        indexer.runStopIndexer()));

        Supplier<Command> shortShoot = () -> Commands.sequence(
                        shooter.runSetRequestedSpeed(() -> ShooterPreferences.SHORT),
                        AutoFire.autonomous(shooter, indexer,
                                        () -> ShooterPreferences.INDEXER_VELOCITY))
                        .withTimeout(10)
                        .andThen(Commands.sequence(
                                        indexer.runStopIndexer(),
                                        shooter.stopShooter(),
                                        indexer.runStopIndexer()));
        // Supplier<Command> fullShoot = () -> Commands.sequence(
        // shooter.runSetRequestedSpeed(() -> ShooterPreferences.LONG),
        // new AutoFire(shooter, indexer, hopper, () ->
        // ShooterPreferences.INDEXER_VELOCITY)
        // .repeatedly())
        // .withTimeout(7)
        // .andThen(Commands.sequence(
        // indexer.runStopIndexer(),
        // shooter.stopShooter(),
        // indexer.runStopIndexer()));

}
