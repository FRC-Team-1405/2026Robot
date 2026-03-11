// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.MotorSim.PhysicsSim;
import frc.robot.sim.sjc.PhysicsSim_SJC;
import frc.robot.util.GamePeriod;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        GamePeriod.elasticInit();
    }

    private void resetSubsystems() {
        // Reset subsystems — schedule stop commands so their initialize() logic runs
        CommandScheduler.getInstance().schedule(m_robotContainer.shooter.stopShooter());
        CommandScheduler.getInstance().schedule(m_robotContainer.indexer.runStopIndexer());
        CommandScheduler.getInstance().schedule(m_robotContainer.hopper.runStopHopper());
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        m_robotContainer.correctOdometry();
        CommandScheduler.getInstance().run();
        RobotContainer.updateNT();
    }

    @Override
    public void disabledInit() {
        // Resetting subsystems here doesn't work
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        resetSubsystems();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        PhysicsSim.getInstance().run();
    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        GamePeriod.elasticTeleopInit();
        resetSubsystems();
    }

    @Override
    public void teleopPeriodic() {
        GamePeriod.elasticPeriodic();
        m_robotContainer.drivetrain.publishMotorCurrent();
        m_robotContainer.intake.publishMotorCurrents();
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
        PhysicsSim_SJC.getInstance().run();
    }
}
