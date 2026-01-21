// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFire extends SequentialCommandGroup {
  /** Creates a new AutoFire. */
  public AutoFire(Shooter shooterSubsytem, Indexer indexerSubsystem, Supplier<AngularVelocity> velocity) {

    addCommands(
        shooterSubsytem.runShooter(velocity),
        Commands.waitUntil(() -> {
          return shooterSubsytem.isReadyToFire();
        }),
        indexerSubsystem.runIndexer().until(() -> {
          return !shooterSubsytem.isReadyToFire();
        }),
        indexerSubsystem.runStopIndexer());
  }
}
