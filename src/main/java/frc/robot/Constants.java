// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.DoubleSolenoidConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double NEO_MAX_RPM = 5700.0; // Max allowed RPM of the NEO motors

    public static final class DriveConstants {

        public static final int LEFT_FRONT_MOTOR_ID = 11;
        public static final int LEFT_BACK_MOTOR_ID = 12;
        public static final int RIGHT_FRONT_MOTOR_ID = 13;
        public static final int RIGHT_BACK_MOTOR_ID = 14;

        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;

        public static final double SLOW_SPEED_MULTIPLIER = 0.5;

        public static final double ACCEL_RATE_LIMIT_1 = 2;
        public static final double ACCEL_RATE_LIMIT_2 = 1.75;

        public static final double WHEEL_SIZE_INCHES = 4.0;
        public static final double DRIVETRAIN_RATIO = 12.75; // Represents gear ratio on WCP Flipped Gearboxes
        public static final double DRIVE_TRACKWIDTH_INCHES = 14.75; // Rough distance between wheels
        public static final double MAX_RPM = 5700.0; // Max allowed RPM of the NEO motors (yes its actually 5700 but
                                                     // we're being conservative)
        public static final double MAX_ANGULAR_VELOCITY = 3 * (2 * Math.PI); // 3 * 2pi radians per second a.k.a. 3
                                                                             // rotation per second

        public static final double RAMSETE_B = 2.0; // Ramsete B constant, 2.0 is the default WPILib value
        public static final double RAMSETE_ZETA = 0.7; // Ramsete Zeta constant, 0.7 is the default WPILib value

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                Units.inchesToMeters(DRIVE_TRACKWIDTH_INCHES));

        /**
         * Conversion factor from encoder rotations to meters
         */
        public static final double DRIVE_ENCODER_POSITION_FACTOR = (Units.inchesToMeters(WHEEL_SIZE_INCHES) * Math.PI)
                / (DRIVETRAIN_RATIO);
        /**
         * Conversion factor from encoder rotations per minute to meters per second
         */
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60.0;

        // Global constraints object for PathPlanner Trajectories
        public static final PathConstraints kPathConstraints = new PathConstraints(1, 0.5);

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

        public static final int[] ENCODER_DIO_PINS = { 1, 2 }; // Encoder wired into DIO pins 1 & 2 on the roboRIO
    }

    public static final class ArmConstants {
        public static final int LIFT_CAN_ID = 24;
        public static final int TELESCOPE_CAN_ID = 22;

        public static final boolean LIFT_INVERTED = false;
        public static final boolean TELESCOPE_INVERTED = false;

        public static final DoubleSolenoidConstants ARM_BRAKE_SOLENOID = new DoubleSolenoidConstants(15, 14);

        public static final double LIFT_GEAR_RATIO = 125; // Represents 125:1 gear ratio on arm lift motor
        public static final double TELESCOPE_GEAR_RATIO = 5; // Represents 5:1 gear ratio on arm telescope motor

        // Convert from encoder rotations to radians
        public static final double LIFT_POSITION_FACTOR = (2 * Math.PI) / LIFT_GEAR_RATIO;
        // TODO: Update to represent telescope position in meters (currently represents
        // rotations in radians)
        public static final double TELESCOPE_POSITION_FACTOR = (2 * Math.PI) / TELESCOPE_GEAR_RATIO;

        /*
         * Arm Horizontal Offset; the Arm Feedforward assumes that the arm's zero
         * position is paralell to the floor. However, the bottom of the arm's travel is
         * lower than this.
         * This offset is set to the encoder when we home the arm.
         * TODO: This must be found and set
         */
        public static final double LIFT_HOME_OFFSET = -(Units.degreesToRadians(30));

        // If set, this is the soft limit for the arm as it flips over the robot
        public static final float LIFT_END_LIMIT = (float) Units.degreesToRadians(225);

        public static final double DEFAULT_LIFT_SPEED = 0.4;
        public static final double DEFAULT_TELESCOPE_SPEED = 0.25;

        public static final double LIFT_INPUT_SCALAR = 0.45;
        public static final double TELESCOPE_INPUT_SCALAR = 0.45;

        public static enum LiftPositions {
            Intake(0.81),
            ScoreLow(0.59),
            ScoreMid(0.59),
            ScoreHigh(0.59);

            public final double val;

            private LiftPositions(double val) {
                this.val = val;
            }
        }
    }

    public static final class IntakeConstants {
        public static final int INTAKE_DRIVE_CAN_ID = 21;

        private static final int INTAKE_OPEN_PORT = 13;
        private static final int INTAKE_CLOSE_PORT = 12;
        public static final DoubleSolenoidConstants INTAKE_OPEN_CLOSE = new DoubleSolenoidConstants(INTAKE_OPEN_PORT,
                INTAKE_CLOSE_PORT);
    }

    public static final class PneumaticConstants {
        // CAN ID for CTRE PCM / Rev PCH
        public static final int P_HUB_CAN_ID = 4;

        public static final DoubleSolenoidConstants LIFT_FEET_CONSTANTS = new DoubleSolenoidConstants(11, 10);
    }

    public static final class LEDConstants {
        public static final int BLINKIN_PWM_PORT = 0;
    }

    public static final class BlinkinConstants {
        public static enum BlinkinState {
            /* --------- FIXED PATTERNS ---------- */
            /* Rainbow & Misc Patterns */
            Rainbow(-0.99),
            PartyRainbow(-0.97),
            OceanRainbow(-0.95),
            LaveRainbow(-0.93),
            ForestRainbow(-0.91),
            RainbowWithGlitter(-0.89),
            Confetti(-0.87),
            /* Shot Pattern */
            RedShot(-0.85),
            BlueShot(-0.83),
            WhiteShot(-0.81),
            /* Sinelon Patterns */
            RainbowSinelon(-0.79),
            PartySinelon(-0.77),
            OceanSinelon(-0.75),
            LaveSinelon(-0.73),
            ForestSinelon(-0.71),
            /* Beats Per Minute Patterns */
            RainbowBPM(-0.69),
            PartyBPM(-0.67),
            OceanBPM(-0.65),
            LaveBPM(-0.63),
            ForestBPM(-0.61),
            /* Fire */
            MediumFire(-0.59),
            LargeFire(-0.57),
            /* Twinkle Patterns */
            RainbowTwinkles(-0.55),
            PartyTwinkles(-0.53),
            OceanTwinkles(-0.51),
            LaveTwinkles(-0.49),
            ForestTwinkles(-0.47),
            /* Colorwave Patterns */
            RainbowColorWaves(-0.45),
            PartyColorWaves(-0.43),
            OceanColorWaves(-0.41),
            LaveColorWaves(-0.39),
            ForestColorWaves(-0.37),
            /* Larson Scanners */
            LarsonScannerRed(-0.35),
            LarsonScannerGray(-0.33),
            /* LightChase Patterns */
            LightChaseRed(-0.31),
            LightChaseBlue(-0.29),
            LightChaseGray(-0.27),
            /* Heartbeat Patterns */
            HeartbeatRed(-0.25),
            HearbeatBlue(-0.23),
            HeartbeatWhite(-0.21),
            HeartbeatGray(-0.19),
            /* Breating Patterns */
            BreathRed(-0.17),
            BreathBlue(-0.15),
            BreathGray(-0.13),
            /* Strobes */
            StrobeRed(-0.11),
            StrobeBlue(-0.09),
            StrobeGold(-0.07),
            StrobeWhite(-0.05),

            /* --------- COLOR 1 PATTERNS ---------- */
            /** Color 1: End to End Blend to Black */
            Color1_E2EBlendBlack(-0.03),
            Color1_LarsonScanner(-0.01),
            Color1_LightChase(0.01),
            Color1_HeartbeatSlow(0.03),
            Color1_HeartbeatMed(0.05),
            Color1_HeartbeatFast(0.07),
            Color1_BreathSlow(0.09),
            Color1_BreathFast(0.11),
            Color1_Shot(0.13),
            Color1_Strobe(0.15),

            /* --------- COLOR 1 PATTERNS ---------- */
            /** Color 1: End to End Blend to Black */
            Color2_E2EBlendBlack(0.17),
            Color2_LarsonScanner(0.19),
            Color2_LightChase(0.21),
            Color2_HeartbeatSlow(0.23),
            Color2_HeartbeatMed(0.25),
            Color2_HeartbeatFast(0.27),
            Color2_BreathSlow(0.29),
            Color2_BreathFast(0.31),
            Color2_Shot(0.33),
            Color2_Strobe(0.35),

            /* --------- Color 1 & 2 Patterns ---------- */
            /** Sparkle, Color 1 on Color 2 */
            Sparkle_1on2(0.37),
            /** Sparkle, Color 2 on Color 1 */
            Sparkle_2on1(0.39),
            /** Color Gradient, Color 1 and 2 */
            Gradient(0.41),
            /** Beats Per Minute, Color 1 and 2 */
            BeatsPerMinute(0.43),
            /** End to End Blend, Color 1 to 2 */
            EndToEndBlend_1to2(0.45),
            /** End to End Blend */
            EndToEndBlend(0.47),
            /**
             * Color 1 and Color 2 no blending <br>
             * </br>
             * <b>Setup Pattern</b>
             */
            Color1and2(0.49),
            SetupPattern(0.49),
            /** Twinkles, Color 1 and 2 */
            Twinkles(0.51),
            /** Color Waves, Color 1 and 2 */
            ColorWaves(0.53),
            /** Sinelon, Color 1 and 2 */
            Sinelon(0.55),

            /* --------- Solid Colors ---------- */
            HotPink(0.57),
            DarkRed(0.59),
            Red(0.61),
            RedOrange(0.63),
            Orange(0.65),
            Gold(0.67),
            Yellow(0.69),
            LawnGreen(0.71),
            Lime(0.73),
            DarkGreen(0.75),
            Green(0.77),
            BlueGreen(0.79),
            Aqua(0.81),
            SkyBlue(0.83),
            DarkBlue(0.85),
            Blue(0.87),
            BlueViolet(0.89),
            Violet(0.91),
            White(0.93),
            Gray(0.95),
            DarkGray(0.97),
            Black(0.99),
            Off(0.99);

            public final double val;

            private BlinkinState(double val) {
                this.val = val;
            }
        }
    }

}
