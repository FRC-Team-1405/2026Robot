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
import frc.robot.Constants;
import frc.robot.Constants.CANBus;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.sim.SimProfiles;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(CANBus.INDEXER_MOTOR);

    private boolean isIndexerActive = false;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    private final NeutralOut m_Brake = new NeutralOut();

    private void setIndexerSpeed(Supplier<AngularVelocity> speed) {
        isIndexerActive = true;
        indexerMotor.setControl(velocityVoltage.withVelocity(speed.get()));
    }

    private void setIndexerSpeed() {
        isIndexerActive = true;
        indexerMotor.setControl(velocityVoltage.withVelocity(Constants.ShooterPreferences.INDEXER_VELOCITY));
    }

    private void indexerStop() {
        isIndexerActive = false;
        indexerMotor.setControl(m_Brake);
    }

    /** Creates a new Indexer. */
    public Indexer() {
        SimProfiles.initIndexer(indexerMotor);
    }

    public Command runIndexer(Supplier<AngularVelocity> speed) {
        return startEnd(() -> setIndexerSpeed(speed), () -> indexerStop());
    }

    public Command runIndexer() {
        return startEnd(() -> setIndexerSpeed(), () -> indexerStop());
    }

    public Command runStopIndexer() {
        return runOnce(this::indexerStop);
    }

    public boolean isIndexerRunning() {
        return isIndexerActive;
    }

    @Override
    public void periodic() {
    }
}
