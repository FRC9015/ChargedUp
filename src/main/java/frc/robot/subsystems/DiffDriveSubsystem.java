package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Dashboard;
import frc.robot.Helpers;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.UpdatePIDFConstantsCommand;
import frc.robot.utils.PIDFConstants;

public class DiffDriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are
    // initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DiffDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DiffDriveSubsystem INSTANCE = new DiffDriveSubsystem();

    /**
     * Returns the Singleton instance of this DiffDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DiffDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DiffDriveSubsystem getInstance() {
        return INSTANCE;
    }

    private final CANSparkMax leftFront, leftBack;
    private final CANSparkMax rightFront, rightBack;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final SparkMaxPIDController leftPID, rightPID;
    private final RamseteController trajRamsete;
    private final DifferentialDriveOdometry odometry;
    private final PIDFConstants velocityPIDFConstants;

    private final Dashboard dash = Dashboard.getInstance();

    private final SimpleWidget leftSpeed = dash.putNumber("Speed/Left", 0), leftSpeedActual = dash.putNumber("Speed/LeftActual", 0);
    private final SimpleWidget rightSpeed = dash.putNumber("Speed/Right", 0), rightSpeedActual = dash.putNumber("Speed/RightActual", 0);

    private final Field2d field;

    /**
     * BiConsumer function that accepts a left and right double values for meters
     * per second
     */
    private BiConsumer<Double, Double> ramseteOutputBiConsumer;

    private IdleMode brakeMode = IdleMode.kCoast;

    private final SlewRateLimiter accelRateLimit1, accelRateLimit2;

    private ArrayList<CANSparkMax> allMotors = new ArrayList<CANSparkMax>();

    private PigeonSubsystem pigeon = PigeonSubsystem.getInstance();

    /**
     * Creates a new instance of this DiffDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DiffDriveSubsystem() {
        CANSparkMax.enableExternalUSBControl(false);

        MotorType motorType = MotorType.kBrushless;
        leftFront = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, motorType);
        allMotors.add(leftFront);
        leftBack = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, motorType);
        allMotors.add(leftFront);

        rightFront = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        allMotors.add(rightFront);
        rightBack = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        allMotors.add(rightBack);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        // Properly invert motors
        leftFront.setInverted(DriveConstants.LEFT_INVERTED);
        rightFront.setInverted(DriveConstants.RIGHT_INVERTED);

        leftEncoder = leftFront.getEncoder();
        leftEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        leftEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        rightEncoder = rightFront.getEncoder();
        rightEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        leftPID = leftFront.getPIDController();

        rightPID = rightFront.getPIDController();

        // Create a new PIDFConstants object for the drive
        velocityPIDFConstants = new PIDFConstants(0.05, 0.0, 0.0, 0, 0.099);

        for (CANSparkMax ctrl : allMotors) {
            ctrl.clearFaults();
            System.out.println(ctrl.getStickyFaults());
            ctrl.burnFlash();
            ctrl.setIdleMode(brakeMode);
        }

        double kMaxOutput = 1;
        double kMinOutput = -1;
        
        velocityPIDFConstants.updateSparkMax(leftPID);
        leftPID.setOutputRange(kMinOutput, kMaxOutput);

        velocityPIDFConstants.updateSparkMax(rightPID);
        rightPID.setOutputRange(kMinOutput, kMaxOutput);

        Dashboard.getInstance().putSendable("Drive Velocity PIDF/Constants", velocityPIDFConstants);
        Dashboard.getInstance().putSendable("Drive Velocity PIDF/Update",
                new UpdatePIDFConstantsCommand(velocityPIDFConstants, leftPID, rightPID));

        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(),
                leftEncoder.getPosition(), rightEncoder.getPosition());
        field = new Field2d();
        //addChild("Field", field);
        //Dashboard.getInstance().putSendable("field", field);

        SmartDashboard.putData("field",field);

        /**
         * Each input to be rate limited must have it's own filter. In any given drive,
         * we have two possible inputs, and thus two filters.
         * 1: used for the Left input (Tank) and the Forward input (Arcade)
         * 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT_1);
        accelRateLimit2 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT_2);

        trajRamsete = new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA);

        ramseteOutputBiConsumer = (left, right) -> {
            setSpeeds(new DifferentialDriveWheelSpeeds(left, right));
        };
    }

    @Override
    public void periodic() {
        Rotation2d rot2d = new Rotation2d(pigeon.getRotation2d().getRadians() - Math.PI);
        odometry.update(rot2d, leftEncoder.getPosition(),
                rightEncoder.getPosition());
        field.setRobotPose(odometry.getPoseMeters());

        leftSpeedActual.getEntry().setDouble(leftEncoder.getVelocity());
        rightSpeedActual.getEntry().setDouble(rightEncoder.getVelocity());
    }

    public Command getPIDFUpdateCommand() {
        return new UpdatePIDFConstantsCommand(velocityPIDFConstants, leftPID, rightPID);
    }

    public void arcadeDrive(double fwd, double turn) {
        arcadeDriveRaw(fwd, turn, true);
    }

    public void tankDrive(double left, double right) {
        tankDriveRaw(left, right, true);
    }

    /**
     * 
     * @param fwd         Forward Speed
     * @param turn        Turning Speed
     * @param rateLimited Should the inputs be rate limited?
     */
    public void arcadeDriveRaw(double fwd, double turn, boolean rateLimited) {
        fwd = rateLimited ? accelRateLimit1.calculate(fwd) : fwd;
        turn = rateLimited ? accelRateLimit2.calculate(turn) : turn;

        double fwdSpeed = calcMetersPerSecond(fwd);
        double turnSpeed = calcRadiansPerSecond(turn);

        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.KINEMATICS
                .toWheelSpeeds(new ChassisSpeeds(fwdSpeed, 0, turnSpeed));

        setSpeeds(wheelSpeeds);
    }

    /**
     * Tank Drive the robot without any speed multiplier and optional rate limiting
     * 
     * @param left        Left Speed
     * @param right       Right Speed
     * @param rateLimited Should the input be rate limited?
     */
    public void tankDriveRaw(double left, double right, boolean rateLimited) {
        left = rateLimited ? accelRateLimit1.calculate(left) : left;
        right = rateLimited ? accelRateLimit2.calculate(right) : right;

        double leftSpeed = calcMetersPerSecond(left);
        double rightSpeed = calcMetersPerSecond(right);

        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);

        setSpeeds(wheelSpeeds);
    }

    public synchronized void setBrakeMode(IdleMode newBrakeMode) {
        this.brakeMode = newBrakeMode;
        for (CANSparkMax controller : allMotors) {
            controller.setIdleMode(newBrakeMode);
        }
    }

    public void stop() {
        setSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }

    public void brakeStop() {
        IdleMode prevBrakeMode = this.brakeMode;
        setBrakeMode(IdleMode.kBrake);
        stop();
        setBrakeMode(prevBrakeMode);
    }
    
    public void resetOdometry(Pose2d initPose) {
        odometry.resetPosition(pigeon.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);
    }

    public void logPosition(String name) {
        Helpers.logBox(
            "Note: " + name,
            "Left Enc: " + leftEncoder.getPosition(), "Right Enc: " + rightEncoder.getPosition(),
            "Rotation2D: " + pigeon.getRotation2d().getDegrees(),
            "Pose2D: " + odometry.getPoseMeters()
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }
    
    public Command getTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            logPosition("BeforePPReset");
            if (isFirstPath) {
                resetOdometry(traj.getInitialPose());
                logPosition("AfterPPReset");
            }
        }),
                /*
                 * Note: currently does not perform transforms based on Alliance Color
                 *
                 * We do not supply PIDControllers or Feedforwards, instead we take the
                 * velocity output from the RamseteController which is then processed by the
                 * SparkMax's built in PID and Feedforward
                 * functionality
                 */
                new PPRamseteCommand(traj, this::getPose, trajRamsete, DriveConstants.KINEMATICS,
                        ramseteOutputBiConsumer, false,
                        this));
    }


    // ----------------- PRIVATE HELPER METHODS ----------------- //

    /**
     * Takes in a joystick input and converts it to meters per second, taking into
     * account slow mode
     * 
     * @param input Joystick input, within range [-1, 1]
     * @return meters per second
     */
    private double calcMetersPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        input = Helpers.calcDeadzone(input, 0.1);

        double inputMetersPerSecond = (input * DriveConstants.MAX_RPM) * DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        // If the robot should be running in slow mode, reduce speed by the multiplier
        // (set in dashboard)
        return isSlowed ? inputMetersPerSecond * speedMultiplier : inputMetersPerSecond;
    }

    private double calcRadiansPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        input = Helpers.calcDeadzone(input, 0.1);

        double inputRadiansPerSecond = input * DriveConstants.MAX_ANGULAR_VELOCITY;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        return isSlowed ? inputRadiansPerSecond * speedMultiplier : inputRadiansPerSecond;
    }

    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        leftPID.setReference(speeds.leftMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(speeds.rightMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        leftSpeed.getEntry().setDouble(speeds.leftMetersPerSecond);
        rightSpeed.getEntry().setDouble(speeds.rightMetersPerSecond);
    }

    private void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

}
