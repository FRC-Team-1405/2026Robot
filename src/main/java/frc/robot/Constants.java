package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class Constants {
    public static class CANBus {
        public static final int SHOOTER_MOTOR_1 = 20;
        public static final int SHOOTER_MOTOR_2 = 21;
    }

    public static class Prefs {
        public static final AngularVelocity SHORT;
        public static final AngularVelocity LONG;

        public static final AngularVelocity INDEXER_VELOCITY;

        public static final double TIGHT;
        public static final double WIDE;
        public static final int STABLE_COUNT;

        public static AngularVelocity distanceToVelocity(Distance distance) {
            double temp = distance.in(Feet);
            return RotationsPerSecond.of(temp * 5);
        }

        static {
            // Shooter Velocities
            Preferences.initDouble("ShooterVelocities/Short", 10);
            SHORT = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Short", 10));
            Preferences.initDouble("ShooterVelocities/Long", 50);
            LONG = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Long", 50));

            // Indexer Velocities
            Preferences.initDouble("IndexerVelocities/IndexerVelocity", 3);
            INDEXER_VELOCITY = RotationsPerSecond.of(Preferences.getDouble("IndexerVelocities/IndexerVelocity", 3));

            Preferences.initDouble("ShooterAccuracy/Tight", 1);
            TIGHT = Preferences.getDouble("ShooterAccuracy/Tight", 1);
            Preferences.initDouble("ShooterAccuracy/Wide", 10);
            WIDE = Preferences.getDouble("ShooterAccuracy/Wide", 10);
            Preferences.initInt("ShooterAccuracy/StableCount", 5);
            STABLE_COUNT = Preferences.getInt("ShooterAccuracy/StableCount", 5);
        }
    }
}
