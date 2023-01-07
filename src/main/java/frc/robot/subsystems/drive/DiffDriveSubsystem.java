package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    Constants.DriveConstants driveConstants = new Constants.DriveConstants();

    private final MotorControllerGroup left;
    private final CANSparkMax left1;
    private final CANSparkMax left2;
    private final MotorControllerGroup right;
    private final CANSparkMax right1;
    private final CANSparkMax right2;
    private final DifferentialDrive drive;

    private IdleMode brakeMode = IdleMode.kCoast;

    /**
     * Creates a new instance of this DiffDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DiffDriveSubsystem() {
        MotorType motorType = MotorType.kBrushless;
        left1 = new CANSparkMax(driveConstants.LEFT_FRONT_MOTOR_ID, motorType);
        left2 = new CANSparkMax(driveConstants.LEFT_BACK_MOTOR_ID, motorType);
        left = new MotorControllerGroup(left1, left2);

        right1 = new CANSparkMax(driveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        right2 = new CANSparkMax(driveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        right = new MotorControllerGroup(right1, right2);

        // Properly invert motors
        left.setInverted(driveConstants.LEFT_INVERTED);
        right.setInverted(driveConstants.RIGHT_INVERTED);

        // Instantiate the drive class
        drive = new DifferentialDrive(left, right);
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }


    public void arcadeDrive(double fwd, double turn) {
        drive.arcadeDrive(fwd, turn);
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
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

}

