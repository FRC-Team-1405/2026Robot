// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AutoFire {

  private AutoFire() {
  }

  /**
   * Teleop fire command. Spins up the flywheel, waits for the first lock, then
   * feeds the indexer continuously until cancelled by button release. The hopper
   * is driven by the hopper-trigger (follows indexer state) so it is NOT
   * required here — claiming it would conflict with the trigger's commands.
   * Designed for {@code whileTrue}.
   */
  public static Command teleop(
      Shooter shooter,
      Indexer indexer,
      Supplier<AngularVelocity> indexerVelocity) {
    return new TeleopFireCommand(shooter, indexer, indexerVelocity);
  }

  /**
   * Autonomous single-shot fire sequence. Spins up, waits for lock, fires one
   * ball, then stops.
   */
  // TODO change this to a time based shooting sequence
  public static Command autonomous(
      Shooter shooter,
      Indexer indexer,
      Hopper hopper,
      Supplier<AngularVelocity> indexerVelocity) {
    return new SequentialCommandGroup(
        shooter.runShooter(),
        Commands.waitUntil(shooter::isReadyToFire),
        indexer.runIndexer(indexerVelocity),
        Commands.waitUntil(() -> !shooter.isReadyToFire()),
        indexer.runStopIndexer(),
        hopper.runStopHopper());
  }

  /**
   * Continuous fire command for teleop use. Spins up the flywheel, waits for
   * the first lock, then feeds the indexer continuously until the command is
   * cancelled (button release). The hopper is managed by the hopper-trigger
   * (follows indexer state) and is intentionally NOT required here to avoid
   * subsystem conflicts with the trigger's commands.
   */
  private static class TeleopFireCommand extends Command {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Supplier<AngularVelocity> indexerVelocity;
    private boolean feeding;

    TeleopFireCommand(Shooter shooter, Indexer indexer,
        Supplier<AngularVelocity> indexerVelocity) {
      this.shooter = shooter;
      this.indexer = indexer;
      this.indexerVelocity = indexerVelocity;
      addRequirements(shooter, indexer);
      setName("AutoFire_Teleop");
    }

    @Override
    public void initialize() {
      shooter.spinUp();
      feeding = false;
      System.out.println("[AutoFire] initialize: spinning up");
    }

    @Override
    public void execute() {
      if (!feeding && shooter.isReadyToFire()) {
        indexer.startFeeding(indexerVelocity);
        feeding = true;
        System.out.println("[AutoFire] locked - feeding started");
      } else if (!shooter.isReadyToFire() && feeding) {
        indexer.stopFeeding();
        feeding = false;
        System.out.println("[AutoFire] unlocked - feeding stopped");
      }
    }

    @Override
    public void end(boolean interrupted) {
      indexer.stopFeeding();
      System.out.println("[AutoFire] end: interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
