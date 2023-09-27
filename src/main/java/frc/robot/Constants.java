// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.robot.utils.DoubleSolenoidConstants;

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

        public static final int LEFT_FRONT_MOTOR_ID = 11;
        public static final int LEFT_BACK_MOTOR_ID = 12;
        public static final int RIGHT_FRONT_MOTOR_ID = 13;
        public static final int RIGHT_BACK_MOTOR_ID = 14;

        public static final int VELOCITY_PID_SLOT = 1; // Slot numbers for PIDF constants
        public static final int POSITION_PID_SLOT = 2; // Slot numbers for PIDF constants

        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;

        public static final double SLOW_SPEED_MULTIPLIER = 0.2;

        public static final double ACCEL_RATE_LIMIT_1 = 2.0;
        public static final double ACCEL_RATE_LIMIT_2 = 4.0;

        public static final double WHEEL_SIZE_INCHES = 6.0;
        public static final double DRIVETRAIN_RATIO =
                12.75; // Represents KOP-included 8.45:1 gear ratio
        public static final double DRIVE_TRACKWIDTH_INCHES = 16; // Rough distance between wheels
        public static final double MAX_RPM =
                5500.0; // Max allowed RPM of the NEO motors (yes its actually 5700 but we're being
        // conservative)
        public static final double MAX_ANGULAR_VELOCITY =
                1.2 * (2 * Math.PI); // 4 * 2pi radians per second a.k.a. 4 rotation per second

        public static final double RAMSETE_B =
                2.0; // Ramsete B constant, 2.0 is the default WPILib value
        public static final double RAMSETE_ZETA =
                0.7; // Ramsete Zeta constant, 0.7 is the default WPILib value

        public static final DifferentialDriveKinematics KINEMATICS =
                new DifferentialDriveKinematics(Units.inchesToMeters(DRIVE_TRACKWIDTH_INCHES));

        /** Conversion factor from encoder rotations to meters */
        public static final double DRIVE_ENCODER_POSITION_FACTOR =
                (Units.inchesToMeters(WHEEL_SIZE_INCHES) * Math.PI) / (DRIVETRAIN_RATIO);
        /** Conversion factor from encoder rotations per minute to meters per second */
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
                DRIVE_ENCODER_POSITION_FACTOR / 60.0;

        // Global constraints object for PathPlanner Trajectories
        public static final PathConstraints kPathConstraints = new PathConstraints(3, 1);
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
        public static final String AUTO_PATH_LAYOUT_NAME = "AutoPath";
    }

    public static final class CounterweightConstants {
        public static final int DRIVE_MOTOR_CAN_ID = 26;
        public static final int ENDSTOP_SWITCH_PORT = 0;

        public static final int[] ENCODER_DIO_PINS = {
            1, 2
        }; // Encoder wired into DIO pins 1 & 2 on the roboRIO
    }

    public static final class ArmConstants {
        public static final int ROTATE_CAN_ID = 24;
        public static final int TELESCOPE_CAN_ID = 22;

        public static final double STAGE_ONE_LENGTH_METERS = 0.64;
        public static final double STAGE_TWO_LENGTH_METERS = 0.64;
        public static final double ARM_MIN_ROTATE_ANGLE = 45;
        public static final double ARM_MAX_ROTATE_ANGLE = 334;
        public static final double ARM_FORCE_NEWTONS = 30;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_DRIVE_CAN_ID = 21;

        public static final DoubleSolenoidConstants OPEN_CLOSE_SOLENOID =
                new DoubleSolenoidConstants(13, 12);
    }

    public static final class FeederIntakeConstants {
        public static final int FEEDER_INTAKE_DRIVE_CAN_ID =
                87; // These constants are placeholder values TODO FIND THE CANSPARKMAX ID AND PUT
        // IT HERE

        public static final DoubleSolenoidConstants OPEN_CLOSE_SOLENOID =
                new DoubleSolenoidConstants(11, 10);
    }

    public static final class PneumaticConstants {
        public static final int P_HUB_CAN_ID = 4;

        public static final DoubleSolenoidConstants LIFT_FEET_CONSTANTS =
                new DoubleSolenoidConstants(11, 10);
    }
}
