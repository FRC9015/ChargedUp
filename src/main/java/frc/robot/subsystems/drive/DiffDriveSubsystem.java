package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Dashboard;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.UpdatePIDFConstantsCommand;
import frc.robot.subsystems.PigeonSubsystem;
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

    private final CANSparkMax left1, left2;
    private final CANSparkMax right1, right2;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final SparkMaxPIDController leftPID, rightPID;
    private final RamseteController trajRamsete;
    private final DifferentialDriveOdometry odometry;
    private final PIDFConstants velocityPIDFConstants, positionPIDFConstants;
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

        MotorType motorType = MotorType.kBrushless;
        left1 = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, motorType);
        allMotors.add(left1);
        left2 = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, motorType);
        left2.follow(left1);
        allMotors.add(left2);

        right1 = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        allMotors.add(right1);
        right2 = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        right2.follow(right1);
        allMotors.add(right2);

        // Properly invert motors
        left1.setInverted(DriveConstants.LEFT_INVERTED);
        right1.setInverted(DriveConstants.RIGHT_INVERTED);

        leftEncoder = left1.getEncoder();
        leftEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        leftEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        rightEncoder = right1.getEncoder();
        rightEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftPID = left1.getPIDController();
        leftPID.setOutputRange(-1, 1, DriveConstants.VELOCITY_PID_SLOT);
        rightPID = right1.getPIDController();
        rightPID.setOutputRange(-1, 1, DriveConstants.VELOCITY_PID_SLOT);

        // Create a new PIDFConstants object for the drive, using the PIDF controller
        // from the left side as a source of truth
        velocityPIDFConstants = new PIDFConstants(leftPID, DriveConstants.VELOCITY_PID_SLOT);
        positionPIDFConstants = new PIDFConstants(leftPID, DriveConstants.POSITION_PID_SLOT);

        addChild("Drive Velocity PIDF Constants", velocityPIDFConstants);
        addChild("Drive Position PIDF Constants", positionPIDFConstants);
        addChild("Update Velocity PIDF Constants", new UpdatePIDFConstantsCommand(velocityPIDFConstants,
                DriveConstants.VELOCITY_PID_SLOT, leftPID, rightPID));
        addChild("Update Position PIDF Constants", new UpdatePIDFConstantsCommand(positionPIDFConstants,
                DriveConstants.POSITION_PID_SLOT, leftPID, rightPID));

        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(),
                Units.inchesToMeters(leftEncoder.getPosition()), Units.inchesToMeters(rightEncoder.getPosition()));
        field = new Field2d();
        addChild("Field", field);

        /**
         * Each input to be rate limited must have it's own filter. In any given drive,
         * we have two possible inputs, and thus two filters.
         * 1: used for the Left input (Tank) and the Forward input (Arcade)
         * 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
        accelRateLimit2 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);

        trajRamsete = new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA);

        ramseteOutputBiConsumer = (left, right) -> {
            leftPID.setReference(left, CANSparkMax.ControlType.kVelocity, DriveConstants.VELOCITY_PID_SLOT);
            rightPID.setReference(right, CANSparkMax.ControlType.kVelocity, DriveConstants.VELOCITY_PID_SLOT);
        };
    }

    @Override
    public void periodic() {
        odometry.update(pigeon.getRotation2d(), leftEncoder.getPosition(),
                rightEncoder.getPosition());
        field.setRobotPose(odometry.getPoseMeters());
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    public void arcadeDrive(double fwd, double turn) {
        System.out.print(odometry.getPoseMeters().getX());
        System.out.print(" , ");
        System.out.println(odometry.getPoseMeters().getY());
        System.out.println(odometry.getPoseMeters().getRotation().getDegrees());

        arcadeDriveRaw(fwd, turn, false);
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
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(fwd, 0, turn));

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

    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        left1.set(speeds.leftMetersPerSecond);
        right1.set(speeds.rightMetersPerSecond);
        //leftPID.setReference(speeds.leftMetersPerSecond, CANSparkMax.ControlType.kVelocity,
        //        DriveConstants.VELOCITY_PID_SLOT);
        //rightPID.setReference(speeds.rightMetersPerSecond, CANSparkMax.ControlType.kVelocity,
        //        DriveConstants.VELOCITY_PID_SLOT);
    }

    public void setBrakeMode(IdleMode newBrakeMode) {
        this.brakeMode = newBrakeMode;
        for (CANSparkMax controller : allMotors) {
            controller.setIdleMode(newBrakeMode);
        }
    }

    public void stop() {
        leftPID.setReference(0, CANSparkMax.ControlType.kVelocity, DriveConstants.VELOCITY_PID_SLOT);
        rightPID.setReference(0, CANSparkMax.ControlType.kVelocity, DriveConstants.VELOCITY_PID_SLOT);
    }

    public void brakeStop() {
        IdleMode prevBrakeMode = this.brakeMode;
        setBrakeMode(IdleMode.kBrake);
        stop();
        setBrakeMode(prevBrakeMode);
    }

    public void resetOdometry(Pose2d initPose) {
        resetEncoders();
        odometry.resetPosition(pigeon.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            if (isFirstPath) {
                resetOdometry(traj.getInitialPose());
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
                        ramseteOutputBiConsumer,
                        this));
    }

    /**
     * Takes in a joystick input and converts it to meters per second, taking into
     * account slow mode
     * 
     * @param input Joystick input, within range [-1, 1]
     * @return meters per second
     */
    private double calcMetersPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        double inputMetersPerSecond = (input * DriveConstants.MAX_RPM) * DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        // If the robot should be running in slow mode, reduce speed by the multiplier
        // (set in dashboard)
        return isSlowed ? inputMetersPerSecond * speedMultiplier : inputMetersPerSecond;
    }

    private double calcRadiansPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        double inputRadiansPerSecond = input * DriveConstants.MAX_ANGULAR_VELOCITY;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        return isSlowed ? inputRadiansPerSecond * speedMultiplier : inputRadiansPerSecond;
    }

    private void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

}
