// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;

// NOTE: Consider using this command inline, rather than writing a subclass. For
// more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFire extends SequentialCommandGroup {
  /** Creates a new AutoFire. */
  public AutoFire(Shooter shooterSubsytem, Indexer indexerSubsystem,
      Supplier<AngularVelocity> shooterVelocity, Supplier<AngularVelocity> indexerVelocity) {

    addCommands(
        Commands.print("shooter is running"),
        shooterSubsytem.runShooter(shooterVelocity),
        Commands.waitUntil(() -> {
          return shooterSubsytem.isReadyToFire();
        }),
        Commands.print("indexer is running"),
        indexerSubsystem.runIndexer(indexerVelocity),
        Commands.waitUntil(() -> {
          if (!shooterSubsytem.isReadyToFire()) {
            System.out.println("stop");
          }
          return !shooterSubsytem.isReadyToFire();
        }),
        Commands.print("indexer has stopped running"),
        indexerSubsystem.runStopIndexer());

  };
}
