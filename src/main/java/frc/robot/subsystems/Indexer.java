// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus;
import frc.robot.Constants.Prefs;
import frc.robot.sim.SimProfiles;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(CANBus.INDEXER_MOTOR);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    private final NeutralOut m_Brake = new NeutralOut();

    private void setIndexerSpeed(Supplier<AngularVelocity> speed) {
        indexerMotor.setControl(velocityVoltage.withVelocity(speed.get()));
    }

    private void indexerStop() {
        indexerMotor.setControl(m_Brake);
    }

    /** Creates a new Indexer. */
    public Indexer() {
        SimProfiles.initIndexer(indexerMotor);
    }

    public Command runIndexer(Supplier<AngularVelocity> speed) {
        return Commands.runOnce(() -> setIndexerSpeed(speed), this);
    }

    public Command stopIndexer() {
        return Commands.runOnce(() -> indexerStop(), this);
    }

    public Command runStopIndexer() {
        return runOnce(this::indexerStop);
    }

    @Override
    public void periodic() {
    }
}
