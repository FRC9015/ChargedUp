package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

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
    private final CANSparkMax left1;
    private final CANSparkMax left2;
    private final MotorControllerGroup right;
    private final CANSparkMax right1;
    private final CANSparkMax right2;
    private final DifferentialDrive drive;

    private IdleMode brakeMode = IdleMode.kCoast;

    private final SlewRateLimiter accelRateLimit1, accelRateLimit2;

    /**
     * Creates a new instance of this DiffDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DiffDriveSubsystem() {
        MotorType motorType = MotorType.kBrushless;
        left1 = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, motorType);
        left2 = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, motorType);
        left = new MotorControllerGroup(left1, left2);
        addChild("Left Motors", left);

        right1 = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        right2 = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        right = new MotorControllerGroup(right1, right2);
        addChild("Right Motors", right);

        // Properly invert motors
        left.setInverted(DriveConstants.LEFT_INVERTED);
        right.setInverted(DriveConstants.RIGHT_INVERTED);

        // Instantiate the drive class
        drive = new DifferentialDrive(left, right);
        addChild("DiffDrive", drive);

        /**
         * Each input to be rate limited must have it's own filter. In any given drive, we have two possible inputs, and thus two filters.
         * 1: used for the Left input (Tank) and the Forward input (Arcade)
         * 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
        accelRateLimit2 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }


    public void arcadeDrive(double fwd, double turn) {
        fwd = accelRateLimit1.calculate(fwd);
        turn = accelRateLimit2.calculate(turn);

        drive.arcadeDrive(fwd, -turn);
    }

    public void tankDrive(double left, double right) {
        left = accelRateLimit1.calculate(left);
        right = accelRateLimit2.calculate(right);
        
        drive.tankDrive(calcSpeed(left), calcSpeed(right));
    }

    public void setBrakeMode(IdleMode newBrakeMode) {
        this.brakeMode = newBrakeMode;
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
        boolean isSlowed = RobotContainer.getInstance().robotState.getSlowed();

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        // If the robot should be running in slow mode, reduce speed by the multiplier (set in dashboard)
        return isSlowed ? input * speedMultiplier : input;
    }

}

