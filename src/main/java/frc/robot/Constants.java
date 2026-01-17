package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class Constants {
    public static class CANBus {
        public static final int SHOOTER = 20;
    }

    public static class ShooterVelocities {
        public static final AngularVelocity SHORT = RotationsPerSecond.of(5.0);
        public static final AngularVelocity LONG = RotationsPerSecond.of(15.0);
    }
}
