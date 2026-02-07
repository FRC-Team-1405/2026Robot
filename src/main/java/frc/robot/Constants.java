// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class Constants {
    public static class CANBus {
        public static final int CLIMBER_MOTOR = 45;
        public static final int CLIMBER_GRABBER = 37;

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
}
