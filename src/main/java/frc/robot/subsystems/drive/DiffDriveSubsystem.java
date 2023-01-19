package frc.robot.subsystems.drive;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PigeonSubsystem;

public class DiffDriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
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

    private final MotorControllerGroup left;
    private final CANSparkMax left1, left2;
    private final MotorControllerGroup right;
    private final CANSparkMax right1, right2;    
    private final DifferentialDrive drive;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

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
        allMotors.add(left2);
        left = new MotorControllerGroup(left1, left2);
        addChild("Left Motors", left);

        right1 = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        allMotors.add(right1);
        right2 = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        allMotors.add(right2);
        right = new MotorControllerGroup(right1, right2);
        addChild("Right Motors", right);

        // Properly invert motors
        left.setInverted(DriveConstants.LEFT_INVERTED);
        right.setInverted(DriveConstants.RIGHT_INVERTED);

        // Instantiate the drive class
        drive = new DifferentialDrive(left, right);
        addChild("DiffDrive", drive);

        // Converts from rotations to wheel position in inches
        double POSITION_CONVERSION_FACTOR = Math.PI * (DriveConstants.WHEEL_SIZE_INCHES / DriveConstants.DRIVETRAIN_RATIO);

        leftEncoder = left1.getEncoder(); 
        leftEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        leftEncoder.setInverted(DriveConstants.LEFT_INVERTED);

        rightEncoder = right1.getEncoder();
        rightEncoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        rightEncoder.setInverted(DriveConstants.RIGHT_INVERTED);

        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(),
                Units.inchesToMeters(leftEncoder.getPosition()), Units.inchesToMeters(rightEncoder.getPosition()));
        field = new Field2d();
        addChild("Field", field);

        /**
         * Each input to be rate limited must have it's own filter. In any given drive, we have two possible inputs, and thus two filters.
         * 1: used for the Left input (Tank) and the Forward input (Arcade)
         * 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
        accelRateLimit2 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
    }

    @Override
    public void periodic() {
        odometry.update(pigeon.getRotation2d(), Units.inchesToMeters(leftEncoder.getPosition()),
                Units.inchesToMeters(rightEncoder.getPosition()));
        field.setRobotPose(odometry.getPoseMeters());
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }


    public void arcadeDrive(double fwd, double turn) {
        fwd = accelRateLimit1.calculate(fwd);
        turn = accelRateLimit2.calculate(turn);

        drive.arcadeDrive(calcSpeed(fwd), calcSpeed(turn));
    }

    public void tankDrive(double left, double right) {
        left = accelRateLimit1.calculate(left);
        right = accelRateLimit2.calculate(right);
        
        drive.tankDrive(calcSpeed(left), calcSpeed(right));
    }

    /**
     * 
     * @param fwd Forward Speed
     * @param turn Turning Speed
     * @param rateLimited Should the inputs be rate limited?
      */
    public void arcadeDriveRaw(double fwd, double turn, boolean rateLimited) {
        fwd = rateLimited ? accelRateLimit1.calculate(fwd) : fwd;
        turn = rateLimited ? accelRateLimit2.calculate(turn) : turn;

        drive.arcadeDrive(fwd, turn);
    }

    /**
     * Tank Drive the robot without any speed multiplier and optional rate limiting
     * @param left Left Speed
     * @param right Right Speed
     * @param rateLimited Should the input be rate limited?
      */
    public void tankDriveRaw(double left, double right, boolean rateLimited) {
        left = rateLimited ? accelRateLimit1.calculate(left) : left;
        right = rateLimited ? accelRateLimit2.calculate(right) : right;
        
        drive.tankDrive(left, right);
    }

    public void setBrakeMode(IdleMode newBrakeMode) {
        this.brakeMode = newBrakeMode;
        for(CANSparkMax controller : allMotors) {
            controller.setIdleMode(newBrakeMode);
        }
    }

    public void stop() {
        drive.tankDrive(0,0);
    }

    public void brakeStop() {
        IdleMode prevBrakeMode = this.brakeMode;
        setBrakeMode(IdleMode.kBrake);
        stop();
        setBrakeMode(prevBrakeMode);
    }

    private double calcSpeed(double input) {
        boolean isSlowed = RobotState.getSlowed();

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        // If the robot should be running in slow mode, reduce speed by the multiplier (set in dashboard)
        return isSlowed ? input * speedMultiplier : input;
    }

}

