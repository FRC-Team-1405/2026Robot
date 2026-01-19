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
        public static final int SHOOTER = 20;
    }

    public static class ShooterVelocities {
        public static final AngularVelocity SHORT;
        public static final AngularVelocity LONG;

        public static AngularVelocity distanceToVelocity(Distance distance) {
            double temp = distance.in(Feet);
            return RotationsPerSecond.of(temp * 5);
        }

        static {
            Preferences.initDouble("ShooterVelocities/Short", 0);
            SHORT = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Short", 0));
            Preferences.initDouble("ShooterVelocities/Long", 1);
            LONG = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Long", 1));
        }
    }
}
