// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutoFire;
// import frc.robot.Commands.IndexerShooterStop;
import frc.robot.Constants.Prefs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        private Shooter shooter = new Shooter();
        private Indexer indexer = new Indexer();
        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController joystickTest = new CommandXboxController(1);

        public RobotContainer() {
                configureBindings();
        }

        private void configureBindings() {
                joystickTest.a().toggleOnTrue(shooter.runShooter(() -> {
                        return Prefs.LONG;
                }));

                // joystickTest.x().onTrue(new IndexerShooterStop(shooter, indexer));

                joystickTest.b().toggleOnTrue(indexer.runIndexer(() -> {
                        return Prefs.INDEXER_VELOCITY;
                }));

                joystickTest.y().toggleOnTrue(
                                new AutoFire(shooter, indexer, () -> Prefs.LONG, () -> Prefs.INDEXER_VELOCITY));
        }

        public Command getAutonomousCommand() {
                return Commands.print("do nothing");
        }
}
