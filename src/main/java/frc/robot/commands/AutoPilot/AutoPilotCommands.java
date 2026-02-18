package frc.robot.commands.AutoPilot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.therekrab.autopilot.APConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.AprilTags;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPilotCommands {

        private static final APConstraints bumpConstraints = new APConstraints()
                        .withAcceleration(1.0) // TUNE THIS TO YOUR ROBOT!
                        .withVelocity(1.0)
                        .withJerk(5.0);

        // Rotations
        // Positive rotations are CCW:
        // https://frcdocs.wpi.edu/en/latest/docs/software/advanced-controls/geometry/pose.html#rotation
        public static final Rotation2d CW_30deg = Rotation2d.fromDegrees(-30);
        public static final Rotation2d CCW_30deg = Rotation2d.fromDegrees(30);
        // TODO: Test origin auto
        // Poses

        public static Supplier<Pose2d> offBlueCenter1 = () -> new Pose2d(2, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> offBlueCenter2 = () -> new Pose2d(0.40, 4.85, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> feedingStation = () -> new Pose2d(0.37, 0.5, Rotation2d.fromDegrees(0));

        // Start Poses
        public static Supplier<Pose2d> startRightFaceIn = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> startLeftFaceIn = () -> new Pose2d(3.55, 7.65, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> startRightFaceFront = () -> new Pose2d(3.55, 0.37, Rotation2d.fromDegrees(0));

        // Bump Poses
        public static Supplier<Pose2d> beforeRightBump = () -> new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> onRightBump = () -> new Pose2d(4.5, 2.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> afterRightBump = () -> new Pose2d(5.5, 2.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> beforeLeftBump = () -> new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> onLeftBump = () -> new Pose2d(4.5, 5.5, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> afterLeftBump = () -> new Pose2d(5.5, 5.5, Rotation2d.fromDegrees(0));

        // Shooter Poses
        public static Supplier<Pose2d> blueShootCenter = () -> new Pose2d(3.5, 4, Rotation2d.fromDegrees(180));

        // Center Poses
        public static Supplier<Pose2d> farRightCenter = () -> new Pose2d(8.25, 1, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> farLeftCenter = () -> new Pose2d(8.25, 6, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> farRightLeft = () -> new Pose2d(7.75, 1, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> farLeftLeft = () -> new Pose2d(7.75, 6, Rotation2d.fromDegrees(90));

        // Depot Poses
        public static Supplier<Pose2d> depotFaceIn = () -> new Pose2d(0.40, 6.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> leftOfDepotFaceIn = () -> new Pose2d(0.40, 7, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> midOfDepotFaceIn = () -> new Pose2d(0.40, 6, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> rightOfDepotFaceIn = () -> new Pose2d(0.40, 5.5, Rotation2d.fromDegrees(270));
        public static Supplier<Pose2d> depotFaceOut = () -> new Pose2d(0.40, 6.5, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> leftOfDepotFaceOut = () -> new Pose2d(0.40, 7, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> midOfDepotFaceOut = () -> new Pose2d(0.40, 6, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> rightOfDepotFaceOut = () -> new Pose2d(0.40, 5.5, Rotation2d.fromDegrees(90));

        // Constant Poses
        // Center Pose is 8,4, Blue Center Pose is 2,4,90
        public static Supplier<Pose2d> blueCenter = () -> new Pose2d(2, 4, Rotation2d.fromDegrees(90));
        public static Supplier<Pose2d> centerOfField = () -> new Pose2d(8, 4, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> origin = () -> new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> fourMeters = () -> new Pose2d(4, 0, Rotation2d.fromDegrees(0));
        public static Supplier<Pose2d> preClimbPosition = () -> new Pose2d(2.5, 4, Rotation2d.fromDegrees(180));
        public static Supplier<Pose2d> climbPosition = () -> new Pose2d(1.5, 4, Rotation2d.fromDegrees(180));

        public static void registerCommands(CommandSwerveDrivetrain drivetrain, Climber climber) {
                /* Commands */
                // Uses command suppliers instead of commands so that we can reuse the same
                // command in an autonomous

                Supplier<Command> MoveTo_blueCenter = () -> new AutoPilotCommand.Builder(
                                () -> blueCenter.get(), drivetrain, "MoveTo_blueCenter")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_origin = () -> new AutoPilotCommand.Builder(
                                () -> origin.get(), drivetrain, "MoveTo_origin")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_centerOfField = () -> new AutoPilotCommand.Builder(
                                () -> centerOfField.get(), drivetrain, "MoveTo_centerOfField")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_startRightFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> startRightFaceIn.get(), drivetrain, "MoveTo_startRightFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_startLeftFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> startLeftFaceIn.get(), drivetrain, "MoveTo_startLeftFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_feedingStation = () -> new AutoPilotCommand.Builder(
                                () -> feedingStation.get(), drivetrain, "MoveTo_feedingStation")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_fourMeters = () -> new AutoPilotCommand.Builder(
                                () -> fourMeters.get(), drivetrain, "MoveTo_fourMeters")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_preClimbPosition = () -> new AutoPilotCommand.Builder(
                                () -> preClimbPosition.get(), drivetrain, "MoveTo_preClimbPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_climbPosition = () -> new AutoPilotCommand.Builder(
                                () -> climbPosition.get(), drivetrain, "MoveTo_climbPosition")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Shooter
                Supplier<Command> MoveTo_blueShootCenter = () -> new AutoPilotCommand.Builder(
                                () -> blueShootCenter.get(), drivetrain, "MoveTo_blueShootCenter")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Center Harvest
                Supplier<Command> MoveTo_farRightLeft = () -> new AutoPilotCommand.Builder(
                                () -> farRightLeft.get(), drivetrain, "MoveTo_farRightLeft")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_farLeftLeft = () -> new AutoPilotCommand.Builder(
                                () -> farLeftLeft.get(), drivetrain, "MoveTo_farLeftLeft")
                                .withFlipPoseForAlliance(true)
                                .build();
                // Depot
                Supplier<Command> MoveTo_depotFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> depotFaceIn.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> leftOfDepotFaceIn.get(), drivetrain, "MoveTo_leftOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> midOfDepotFaceIn.get(), drivetrain, "MoveTo_midOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceIn = () -> new AutoPilotCommand.Builder(
                                () -> rightOfDepotFaceIn.get(), drivetrain, "MoveTo_rightOfDepotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_depotFaceOut = () -> new AutoPilotCommand.Builder(
                                () -> depotFaceOut.get(), drivetrain, "MoveTo_depotFaceIn")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_leftOfDepotFaceOut = () -> new AutoPilotCommand.Builder(
                                () -> leftOfDepotFaceOut.get(), drivetrain, "MoveTo_leftOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_midOfDepotFaceOut = () -> new AutoPilotCommand.Builder(
                                () -> midOfDepotFaceOut.get(), drivetrain, "MoveTo_midOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_rightOfDepotFaceOut = () -> new AutoPilotCommand.Builder(
                                () -> rightOfDepotFaceOut.get(), drivetrain, "MoveTo_rightOfDepotFaceOut")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_offBlueCenter1 = () -> new AutoPilotCommand.Builder(
                                () -> offBlueCenter1.get(), drivetrain, "MoveTo_offBlueCenter1")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_offBlueCenter2 = () -> new AutoPilotCommand.Builder(
                                () -> offBlueCenter2.get(), drivetrain, "MoveTo_offBlueCenter2")
                                .withFlipPoseForAlliance(true)
                                .build();
                // right from the driver station view
                // Bump Stuff
                Supplier<Command> MoveTo_beforeRightBump = () -> new AutoPilotCommand.Builder(
                                () -> beforeRightBump.get(), drivetrain, "MoveTo_beforeRightBump")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_onRightBump = () -> new AutoPilotCommand.Builder(
                                () -> onRightBump.get(), drivetrain, "MoveTo_onRightBump")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_afterRightBump = () -> new AutoPilotCommand.Builder(
                                () -> afterRightBump.get(), drivetrain, "MoveTo_afterRightBump")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_beforeLeftBump = () -> new AutoPilotCommand.Builder(
                                () -> beforeLeftBump.get(), drivetrain, "MoveTo_beforeLeftBump")
                                .withFlipPoseForAlliance(true)
                                .build();
                Supplier<Command> MoveTo_onLeftBump = () -> new AutoPilotCommand.Builder(
                                () -> onLeftBump.get(), drivetrain, "MoveTo_onLeftBump")
                                .withFlipPoseForAlliance(true)
                                .withConstraints(bumpConstraints)
                                .build();
                Supplier<Command> MoveTo_afterLeftBump = () -> new AutoPilotCommand.Builder(
                                () -> afterLeftBump.get(), drivetrain, "MoveTo_afterLeftBump")
                                .withFlipPoseForAlliance(true)
                                .build();

                /* Full Autos */ // TODO: DON'T FORGET THE COMMAS
                // Test/Move to a position
                Command AP_blueCenter = new SequentialCommandGroup(
                                MoveTo_blueCenter.get());
                Command AP_origin = new SequentialCommandGroup(
                                MoveTo_origin.get());
                Command AP_ShootFromDepot = new SequentialCommandGroup(
                                MoveTo_blueShootCenter.get());
                Command AP_blueCenterToDepot = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_depotFaceIn.get());
                Command AP_fourMeters = new SequentialCommandGroup(
                                MoveTo_fourMeters.get());
                Command AP_blueShootCenter = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_blueShootCenter.get());
                // Mini Autos
                Command climbCommand = climber.runClimbUp().withName("Auto Climb Up");
                SmartDashboard.putData(climbCommand);
                Command AP_DepotFaceIn = new SequentialCommandGroup(
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get());
                Command AP_rightBump = new SequentialCommandGroup(
                                MoveTo_beforeRightBump.get(),
                                MoveTo_onRightBump.get(),
                                MoveTo_afterRightBump.get());
                Command AP_CenterHarvest = new SequentialCommandGroup(
                                MoveTo_farRightLeft.get(),
                                MoveTo_farLeftLeft.get());
                Command AP_climb = new SequentialCommandGroup(
                                MoveTo_preClimbPosition.get(),
                                climbCommand,
                                MoveTo_climbPosition.get(),
                                climber.runClimbDown(),
                                Commands.print("climbing"));

                // Actual Full autos
                Command AP_LeftStartDepotScore = new SequentialCommandGroup(
                                // MoveTo_blueCenter.get(),
                                MoveTo_startLeftFaceIn.get(),
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_blueShootCenter.get());

                Command AP_RightStartDepotScore = new SequentialCommandGroup(
                                MoveTo_startRightFaceIn.get(),
                                MoveTo_blueCenter.get(),
                                MoveTo_offBlueCenter1.get(),
                                MoveTo_offBlueCenter2.get(),
                                MoveTo_rightOfDepotFaceOut.get(),
                                MoveTo_midOfDepotFaceOut.get(),
                                MoveTo_depotFaceOut.get(),
                                MoveTo_leftOfDepotFaceOut.get(),
                                MoveTo_blueShootCenter.get());
                Command cmd = climber.runExtendClimber().withName("Auto Climb");
                SmartDashboard.putData(cmd);

                Command AP_RightStartFeedingStationScore = new SequentialCommandGroup(
                                MoveTo_startRightFaceIn.get(),
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_blueShootCenter.get(),
                                Commands.parallel(MoveTo_afterLeftBump.get(), cmd));
                // Commands.print("climbing").andThen(Commands.waitSeconds(3)));

                Command AP_RightStartCenterHarvestInLeft = new SequentialCommandGroup(
                                MoveTo_startRightFaceIn.get(),
                                MoveTo_beforeRightBump.get(),
                                MoveTo_onRightBump.get(),
                                MoveTo_afterRightBump.get(),
                                MoveTo_farRightLeft.get(),
                                MoveTo_farLeftLeft.get(),
                                MoveTo_afterLeftBump.get(),
                                MoveTo_onLeftBump.get(),
                                MoveTo_beforeLeftBump.get());
                // TODO:Edit and refine
                Command AP_TheShowboater = new SequentialCommandGroup(
                                MoveTo_blueCenter.get(),
                                MoveTo_startLeftFaceIn.get(),
                                MoveTo_leftOfDepotFaceIn.get(),
                                MoveTo_depotFaceIn.get(),
                                MoveTo_midOfDepotFaceIn.get(),
                                MoveTo_rightOfDepotFaceIn.get(),
                                MoveTo_blueShootCenter.get(),
                                MoveTo_feedingStation.get(),
                                Commands.waitSeconds(Constants.AutonomousPreferences.WAIT_TIME),
                                MoveTo_blueShootCenter.get(),
                                MoveTo_beforeRightBump.get(),
                                MoveTo_onRightBump.get(),
                                MoveTo_afterRightBump.get(),
                                MoveTo_farRightLeft.get(),
                                MoveTo_farLeftLeft.get(),
                                MoveTo_afterLeftBump.get(),
                                MoveTo_onLeftBump.get(),
                                MoveTo_beforeLeftBump.get(),
                                MoveTo_blueShootCenter.get());

                /* Register Commands */ // any auto added here needs to be registered in AutoCommands to show up on
                                        // Elastic
                NamedCommands.registerCommand("AP_blueCenter", AP_blueCenter);
                NamedCommands.registerCommand("AP_blueShootCenter", AP_blueShootCenter);
                NamedCommands.registerCommand("AP_blueCenterToDepot", AP_blueCenterToDepot);
                NamedCommands.registerCommand("AP_fourMeters", AP_fourMeters);
                NamedCommands.registerCommand("AP_DepotFaceIn", AP_DepotFaceIn);
                NamedCommands.registerCommand("AP_ShootFromDepot", AP_ShootFromDepot);
                NamedCommands.registerCommand("AP_origin", AP_origin);
                NamedCommands.registerCommand("AP_climb", AP_climb);
                NamedCommands.registerCommand("AP_rightBump", AP_rightBump);
                NamedCommands.registerCommand("AP_CenterHarvest", AP_CenterHarvest);
                NamedCommands.registerCommand("AP_LeftStartDepotScore", AP_LeftStartDepotScore);
                NamedCommands.registerCommand("AP_RightStartDepotScore", AP_RightStartDepotScore);
                NamedCommands.registerCommand("AP_RightStartFeedingStationScore", AP_RightStartFeedingStationScore);
                NamedCommands.registerCommand("AP_RightStartCenterHarvestInLeft", AP_RightStartCenterHarvestInLeft);
                NamedCommands.registerCommand("AP_TheShowboater", AP_TheShowboater);
        }
}
