package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.PIDFConstants;
public class ArmSubsystem extends SubsystemBase
{
    private static ArmSubsystem INSTANCE;
    
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        if(INSTANCE == null) INSTANCE = new ArmSubsystem();
        return INSTANCE;
    }


    private final CANSparkMax rotateArm; // rotateArm pivots the arm. 
    private final RelativeEncoder rotateEncoder;
    private final SparkMaxPIDController rotatePID;
    private final PIDFConstants rotatePIDConstants;
    private double rotateArmMinPosition;

    private final DoubleSolenoid rotateArmBrake;

    private final CANSparkMax telescopeArm; // telescopeArm moves the arm in and out.
    private final RelativeEncoder telescopeEncoder;
    private final SparkMaxPIDController telescopePID;
    private final PIDFConstants telescopePIDConstants;
    private double telescopeArmMinPosition;

    private ArmSubsystem() 
    {
        // Set up stuff for the Arm rotating motors
        rotateArm = new CANSparkMax(ArmConstants.LIFT_CAN_ID, MotorType.kBrushless);
        rotateArm.setInverted(ArmConstants.LIFT_INVERTED);

        rotateArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

        rotateEncoder = rotateArm.getEncoder();
        homeRotate();

        rotatePID = rotateArm.getPIDController();

        rotatePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1); // TODO: THESE CONSTANTS MUST BE TUNED
        rotatePIDConstants.updateSparkMax(rotatePID);

        // Initialize arm rotation brake and make sure it is released on boot
        rotateArmBrake = PneumaticHubSubsystem.getDoubleSolenoid(ArmConstants.ARM_BRAKE_SOLENOID);
        rotateArmBrake.set(DoubleSolenoid.Value.kReverse);

        // Set up stuff for the Arm Telescoping motor
        telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
        telescopeArm.setInverted(ArmConstants.TELESCOPE_INVERTED);

        telescopeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);

        telescopeEncoder = telescopeArm.getEncoder();
        homeTelescope();
        telescopePID = telescopeArm.getPIDController();

        telescopePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1); // TODO: THESE CONSTANTS MUST BE TUNED
        telescopePIDConstants.updateSparkMax(telescopePID);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    public void rotateArm(double input){
        double inputScaled = MathUtil.applyDeadband(input, 0.05) * ArmConstants.LIFT_INPUT_SCALAR;
        double inputRPM = inputScaled * Constants.NEO_MAX_RPM;
        rotateArmRaw(inputRPM);
    }

    /**
     * DO NOT USE FOR JOYSTICK INPUT.
     * @param rpm
     */
    public void rotateArmRaw(double rpm) {
        rotatePID.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Hold the arm at the current position. <br></br>
     * This gets the current encoder position and sets in the SparkMaxPID with the Position control type.
     */
    public void holdRotatePosition() {
        rotatePID.setReference(getRotatePosition(), ControlType.kPosition);
    }

    /**
     * Home the arm. This retrieves the current position of the arm, sets the min position variable, and sets the soft limit.
     */
    public void homeRotate() {
        rotateArmMinPosition = rotateEncoder.getPosition();
        rotateArm.setSoftLimit(SoftLimitDirection.kReverse, ((float) rotateArmMinPosition));
    }

    /**
     * Send the arm to a preset position
     * @param position
     * @return the actual position it is going to which is the lift position + the home position
     */
    public double rotateToPosition(ArmConstants.LiftPositions position) {
        double actualPos = rotateArmMinPosition + position.val;
        rotatePID.setReference(actualPos, ControlType.kPosition);
        return actualPos;
    }

    public double getTelescopePosition() {
        return telescopeEncoder.getPosition();
    }

    public void telescopeArm(double input){
        double inputScaled = MathUtil.applyDeadband(input, 0.05) * ArmConstants.LIFT_INPUT_SCALAR;
        double inputRPM = inputScaled * Constants.NEO_MAX_RPM;
        telescopeArmRaw(inputRPM);
    }

    /**
     * DO NOT USE FOR JOYSTICK INPUT.
     * @param rpm
     */
    public void telescopeArmRaw(double rpm) {
        telescopePID.setReference(rpm, ControlType.kVelocity);
    }

     /**
     * Hold the arm telescope at the current position. <br></br>
     * This gets the current encoder position and sets in the SparkMaxPID with the Position control type.
     */
    public void holdTelescopePosition() {
        telescopePID.setReference(getTelescopePosition(), ControlType.kPosition);
    }

    /**
     * Home the arm telescope. This retrieves the current position of the arm, sets the min position variable, and sets the soft limit.
     */
    public void homeTelescope() {
        telescopeArmMinPosition = telescopeEncoder.getPosition();
        telescopeArm.setSoftLimit(SoftLimitDirection.kReverse, ((float) telescopeArmMinPosition));
    }

    /**
     * Check if the brake is on the arm
     * @return whether the brake is applied on the arm
     */
    public boolean getBrakeApplied () {
        return rotateArmBrake.get() == DoubleSolenoid.Value.kForward;
    }


    // ----------------- PRIVATE HELPER METHODS ----------------- //

    private void applyBrake() {
        double currentVel = rotateEncoder.getVelocity();

        if (Math.abs(currentVel) < 0.01) {
            rotateArmBrake.set(DoubleSolenoid.Value.kForward);
            rotateArm.setIdleMode(IdleMode.kBrake);
        }
    }

    private void releaseBrake() {
        rotateArm.setIdleMode(IdleMode.kBrake);
        rotateArmBrake.set(DoubleSolenoid.Value.kReverse);
    }
}
