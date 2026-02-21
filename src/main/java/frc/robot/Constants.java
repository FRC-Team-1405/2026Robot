package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
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

        public static final int INDEXER_MOTOR = 22;

        public static final int ADJUSTABLE_SHOOTER_MOTOR = 24;
    }

    public static class Prefs {
        public static final AngularVelocity SHORT;
        public static final AngularVelocity LONG;

        public static final AngularVelocity INDEXER_VELOCITY;

        public static final double TIGHT;
        public static final double WIDE;
        public static final int STABLE_COUNT;

        // Flywheel physical properties
        public static final double FLYWHEEL_DIAMETER_INCHES = 2.25;
        public static final double FLYWHEEL_WEIGHT_LBS = 5.2;

        // Gear ratio: wheel spins 1.5x faster than motor output (3:2 ratio)
        public static final double MOTOR_TO_WHEEL_GEAR_RATIO = 1.5;

        // Moment of inertia calculation for simulation
        // Approximating flywheel as solid cylinder: I = (1/2) * m * r^2
        // Using WPILib Units for conversion:
        // mass = 5.2 lbs, radius = 1.125 inches
        // I = 0.5 * mass_kg * radius_m^2
        private static final double FLYWHEEL_MASS_KG = Pounds.of(FLYWHEEL_WEIGHT_LBS).in(Kilograms);
        private static final double FLYWHEEL_RADIUS_M = Inches.of(FLYWHEEL_DIAMETER_INCHES / 2.0).in(Meters);
        public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.5 * FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M
                * FLYWHEEL_RADIUS_M; // kg*m^2

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
            Preferences.initDouble("IndexerVelocities/IndexerVelocity", 20);
            INDEXER_VELOCITY = RotationsPerSecond.of(Preferences.getDouble("IndexerVelocities/IndexerVelocity", 20));

            Preferences.initDouble("ShooterAccuracy/Tight", 1);
            TIGHT = Preferences.getDouble("ShooterAccuracy/Tight", 1);
            Preferences.initDouble("ShooterAccuracy/Wide", 10);
            WIDE = Preferences.getDouble("ShooterAccuracy/Wide", 10);
            Preferences.initInt("ShooterAccuracy/StableCount", 5);
            STABLE_COUNT = Preferences.getInt("ShooterAccuracy/StableCount", 5);
        }
    }
}
