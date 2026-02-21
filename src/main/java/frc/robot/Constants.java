// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class Constants {
    public static class CANBus {
        public static final int CLIMBER_MOTOR = 45;
        public static final int CLIMBER_GRABBER = 37;
        public static final int HOPPER_MOTOR = 25;
        public static final int INTAKE_MOTOR = 28;
        public static final int PICKUP_MOTOR = 29;

        public static final int SHOOTER_MOTOR_1 = 41;
        public static final int SHOOTER_MOTOR_2 = 42;
        public static final int SHOOTER_MOTOR_3 = 43;

        public static final int INDEXER_MOTOR = 22;

        public static final int ADJUSTABLE_SHOOTER_MOTOR = 24;

    }

    public static class ShooterPreferences {
        public static final AngularVelocity SHORT;
        public static final AngularVelocity INTERMEDIATE;
        public static final AngularVelocity MEDIUM;
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
            Preferences.initDouble("ShooterVelocities/Intermediate", 20);
            INTERMEDIATE = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Intermediate", 20));
            Preferences.initDouble("ShooterVelocities/Medium", 30);
            MEDIUM = RotationsPerSecond.of(Preferences.getDouble("ShooterVelocities/Medium", 30));
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

    public static class ClimberPreferences {
        public static final double CLIMBER_EXTEND_POSITION;
        public static final double CLIMBER_RETRACT_POSITION;
        public static final double GRABBER_OPEN_POSITION;
        public static final double GRABBER_CLOSED_POSITION;
        public static final double POSITION_TOLERANCE;
        public static final double SETTLE_MAX;

        static {
            Preferences.initDouble("Climber/Arm/Extended", 100.0);
            CLIMBER_EXTEND_POSITION = Preferences.getDouble("Climber/Arm/Extended", 100.0);

            Preferences.initDouble("Climber/Arm/Retracted", 0.0);
            CLIMBER_RETRACT_POSITION = Preferences.getDouble("Climber/Arm/Retracted", 0.0);

            Preferences.initDouble("Climber/Grabber/Open", 50.0);
            GRABBER_OPEN_POSITION = Preferences.getDouble("Climber/Grabber/Open", 50.0);

            Preferences.initDouble("Climber/Grabber/Closed", 0.0);
            GRABBER_CLOSED_POSITION = Preferences.getDouble("Climber/Grabber/Closed", 0.0);

            Preferences.initDouble("Climber/PositionTolerance", 1.0);
            POSITION_TOLERANCE = Preferences.getDouble("Climber/PositionTolerance", 1.0);

            Preferences.initDouble("Climber/SettleMax", 3);
            SETTLE_MAX = Preferences.getDouble("Climber/SettleMax", 3);
        }
    }

    public static class HoodPreferences {

        public static final double SERVO_SPEED_SECONDS;
        public static final double SERVO_SHORT;
        public static final double SERVO_MEDIUM;
        public static final double SERVO_LONG;

        static {
            Preferences.initDouble("Hood/Servo Speed Seconds", 0.5);
            SERVO_SPEED_SECONDS = Preferences.getDouble("Hood/Servo Speed Seconds", 0.5);

            Preferences.initDouble("Hood/Servo Speed Short", 0.3);
            SERVO_SHORT = Preferences.getDouble("Hood/Servo Speed Short", 0.3);

            Preferences.initDouble("Hood/Servo Speed Medium", 0.6);
            SERVO_MEDIUM = Preferences.getDouble("Hood/Servo Speed Medium", 0.6);

            Preferences.initDouble("Hood/Servo Speed Long", 1.0);
            SERVO_LONG = Preferences.getDouble("Hood/Servo Speed Long", 1.0);
        }

    }

    public static class AutonomousPreferences {

        public static final double WAIT_TIME;

        static {
            Preferences.initDouble("Auto Wait Time (Seconds)", 3.0);
            WAIT_TIME = Preferences.getDouble("Auto Wait Time (Seconds)", 3.0);

        }

    }

    public static class HopperPreferences {

        public static final AngularVelocity HOPPER_FORWARD_SPEED;
        public static final AngularVelocity HOPPER_REVERSE_SPEED;

        static {
            Preferences.initDouble("Hopper/Forward", 1.0);
            HOPPER_FORWARD_SPEED = RotationsPerSecond.of(Preferences.getDouble("Hopper/Forward", 1.0));

            Preferences.initDouble("Hopper/Reverse", -1.0);
            HOPPER_REVERSE_SPEED = RotationsPerSecond.of(Preferences.getDouble("Hopper/Reverse", -1.0));

        }
    }

    public static class IntakePreferences {
        public static final double INTAKE_MOTOR_OUT;
        public static final double INTAKE_MOTOR_IN;
        public static final double INTAKE_MOTOR_CENTER;
        public static final double PICKUP_MOTOR_OUT;
        public static final double PICKUP_MOTOR_IN;
        public static final double SETTLE_MAX;
        public static final double POSITION_TOLERANCE;
        static {
            Preferences.initDouble("Intake/Out", 70.0);
            INTAKE_MOTOR_OUT = Preferences.getDouble("Intake/Out", 70.0);
            Preferences.initDouble("Intake/In", 3.0);
            INTAKE_MOTOR_IN = Preferences.getDouble("Intake/In", 3.0);
            Preferences.initDouble("Intake/Center", 50.0);
            INTAKE_MOTOR_CENTER = Preferences.getDouble("Intake/Center", 50.0);
            Preferences.initDouble("Pickup/Out", -25.0);
            PICKUP_MOTOR_OUT = Preferences.getDouble("Pickup/Out", -25.0);
            Preferences.initDouble("Pickup/In", 50.0);
            PICKUP_MOTOR_IN = Preferences.getDouble("Pickup/In", 50.0);
            Preferences.initDouble("Intake/Settle Max", 3.0);
            SETTLE_MAX = Preferences.getDouble("Intake/SettleMax", 3.0);
            Preferences.initDouble("Intake/PositionTolerance", 0.5);
            POSITION_TOLERANCE = Preferences.getDouble("Intake/PositionTolerance", 0.5);
        }
    }
}
