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
   * Teleop fire command. Holds the flywheel at the requested RPS and gates the
   * indexer/hopper based on the flywheel's lock state every cycle. Designed for
   * {@code whileTrue} — runs until cancelled by button release.
   */
  public static Command teleop(
      Shooter shooter,
      Indexer indexer,
      Hopper hopper,
      Supplier<AngularVelocity> indexerVelocity) {
    return new TeleopFireCommand(shooter, indexer, hopper, indexerVelocity);
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
   * Continuous fire command for teleop use. A single command with a proper
   * lifecycle — no sequential waits, no .repeatedly(), no extra scheduler
   * overhead. Each execute() cycle checks the flywheel's lock state and toggles
   * the indexer/hopper accordingly, giving sub-20ms reaction time.
   */
  public static class TeleopFireCommand extends Command {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Hopper hopper;
    private final Supplier<AngularVelocity> indexerVelocity;
    private boolean feeding;

    public TeleopFireCommand(Shooter shooter, Indexer indexer, Hopper hopper,
        Supplier<AngularVelocity> indexerVelocity) {
      this.shooter = shooter;
      this.indexer = indexer;
      this.hopper = hopper;
      this.indexerVelocity = indexerVelocity;
      addRequirements(shooter, indexer, hopper);
      setName("AutoFire_Teleop");
    }

    @Override
    public void initialize() {
      shooter.spinUp();
      feeding = false;
    }

    @Override
    public void execute() {
      if (shooter.isReadyToFire() && !feeding) {
        indexer.startFeeding(indexerVelocity);
        hopper.startFeeding();
        feeding = true;
      } else if (!shooter.isReadyToFire() && feeding) {
        indexer.stopFeeding();
        hopper.stopFeeding();
        feeding = false;
      }
    }

    @Override
    public void end(boolean interrupted) {
      shooter.spinDown();
      indexer.stopFeeding();
      hopper.stopFeeding();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
