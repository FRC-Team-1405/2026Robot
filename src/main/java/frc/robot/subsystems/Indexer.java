// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANBus;
import frc.robot.Constants.ShooterPreferences;
import frc.robot.sim.SimProfiles;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor = new TalonFX(CANBus.INDEXER_MOTOR);

    private final MotionMagicVelocityVoltage velocityVoltage = new MotionMagicVelocityVoltage(0);

    private final NeutralOut m_Brake = new NeutralOut();

    private boolean isIndexerActive = false;

    private void setIndexerSpeed(Supplier<AngularVelocity> speed) {
        isIndexerActive = true;
        SmartDashboard.putBoolean("isIndexerActive", isIndexerActive);
        indexerMotor.setControl(velocityVoltage.withVelocity(speed.get()));
    }

    private void setIndexerSpeed() {
        isIndexerActive = true;
        SmartDashboard.putBoolean("isIndexerActive", isIndexerActive);
        indexerMotor.setControl(velocityVoltage.withVelocity(Constants.ShooterPreferences.INDEXER_VELOCITY));
    }

    private void indexerStop() {
        isIndexerActive = false;
        SmartDashboard.putBoolean("isIndexerActive", isIndexerActive);
        indexerMotor.setControl(m_Brake);
    }

    /** Creates a new Indexer. */
    public Indexer() {
        SimProfiles.initIndexer(indexerMotor);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kS = 0.1;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kP = 0.5;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        configs.Voltage.withPeakForwardVoltage(Volts.of(10))
                       .withPeakReverseVoltage(Volts.of(-10));

        configs.MotionMagic.MotionMagicAcceleration = 40;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = indexerMotor.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Indexer config failed: " + status.toString());
        }
    }

    public Command runIndexer(Supplier<AngularVelocity> speed) {
        return runOnce(() -> setIndexerSpeed(speed));
    }

    public Command runIndexer() {
        return runOnce(() -> setIndexerSpeed());
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
