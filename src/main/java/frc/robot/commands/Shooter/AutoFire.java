// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE: Consider using this command inline, rather than writing a subclass. For
// more
// information, see: 
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFire extends SequentialCommandGroup {

  /** Creates a new AutoFire. */
  public AutoFire(Shooter shooterSubsytem, Indexer indexerSubsystem, Hopper hopper,
      Supplier<AngularVelocity> indexerVelocity, DoubleSupplier distanceToHub) {

    addCommands(
        shooterSubsytem.runSetRequestedSpeed(() -> {
          Distance distance = Meters.of(distanceToHub.getAsDouble());
          return Constants.ShooterPreferences.distanceToVelocity(distance);
        }),
        Commands.waitUntil(() -> {
          return shooterSubsytem.isReadyToFire();
        }),
        indexerSubsystem.runIndexer(indexerVelocity),
        Commands.waitUntil(() -> {
          if (!shooterSubsytem.isReadyToFire()) {
            System.out.println("stop");
          }
          return !shooterSubsytem.isReadyToFire();
        }),
        indexerSubsystem.runStopIndexer(),
        hopper.runStopHopper());

  };
}
