// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

        public static final int LEFT_FRONT_MOTOR_ID = 14;
        public static final int LEFT_BACK_MOTOR_ID = 15;
        public static final int RIGHT_FRONT_MOTOR_ID = 16;
        public static final int RIGHT_BACK_MOTOR_ID = 13;

        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final double SLOW_SPEED_MULTIPLIER = 0.5;
        
        public static final double ACCEL_RATE_LIMIT = 0.75;

        public static final double WHEEL_SIZE_INCHES = 4.0;
        public static final double DRIVETRAIN_RATIO = 8.45; // Represents KOP-included 8.45:1 gear ratio

    }

    public static final class SensorConstants {
        public static final int PIGEON_CAN_ID = 6;
    }

    public static final class DashboardConstants {
        public static final String TELEOP_TAB_NAME = "TeleOp";
        public static final String AUTO_TAB_NAME = "Autonomous";

        public static final String BALANCE_LAYOUT_NAME = "Balance";
        public static final String DRIVE_LAYOUT_NAME = "Drive";
        public static final String COUNTERWEIGHT_LAYOUT_NAME = "Counterweight";
    }

    public static final class CounterweightConstants {
        public static final int DRIVE_MOTOR_CAN_ID = 7;
        public static final int ENDSTOP_SWITCH_PORT = 0;
        public static final int[] ENCODER_DIO_PINS = {1, 2}; // Encoder wired into DIO pins 1 & 2 on the roboRIO
    }
    public static final class ArmConstants {
        public static final int ROTATE_CAN_ID=0;
        public static final int TELESCOPE_CAN_ID=0;
    }

}
