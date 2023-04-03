package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.PIDFConstants;
import frc.robot.utils.TelescopingArmFeedforward;
import lombok.Getter;
import lombok.val;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ArmSubsystem();
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

        rotatePID = rotateArm.getPIDController();

        rotatePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1); // TODO: THESE CONSTANTS MUST BE TUNED
        rotatePIDConstants.updateSparkMax(rotatePID);

        // Custom-ish feedforward class
        rotateFF = new TelescopingArmFeedforward(ArmConstants.ROTATE_KS, ArmConstants.ROTATE_MIN_KG,
                ArmConstants.ROTATE_MAX_KG, ArmConstants.ROTATE_KV);

        // ---------- Telescope Motion Setup ---------
        telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
        telescopeArm.restoreFactoryDefaults(false);
        telescopeArm.setInverted(ArmConstants.TELESCOPE_INVERTED);

        telescopeEncoder = telescopeArm.getEncoder();
        telescopePID = telescopeArm.getPIDController();

        telescopePIDConstants = new PIDFConstants(0.1, 0, 0, 0, 0.1); // TODO: THESE CONSTANTS MUST BE TUNED
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

    public void rotateArm(double input) {
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
     * Home the arm. This sets the bottom limit of the arm.
     * 
     * @param setEndLimit If true, set the limit of the arm going over the robot
     */
    public void homeRotate(boolean setEndLimit) {
        rotateEncoder.setPosition(0);
        rotateArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        rotateArm.setSoftLimit(SoftLimitDirection.kReverse, ((float) rotateEncoder.getPosition()));

        rotateArm.enableSoftLimit(SoftLimitDirection.kForward, true);
        rotateArm.setSoftLimit(SoftLimitDirection.kForward, ((float) Units.degreesToRadians(270)));
    }

    /**
     * Send the arm to a preset position
     * 
     * @param position
     */
    public double rotateToPosition(ArmConstants.LiftPositions position) {
        double actualPos = rotateArmMinPosition + position.val;
        rotatePID.setReference(actualPos, ControlType.kPosition);
        return actualPos;
    }

    /* --------- ROTATE CONTROLS --------- */

    public double getTelescopePosition() {
        return telescopeEncoder.getPosition();
    }

    public double getTelescopePercent() {
        val maxPosition = telescopeArm.getSoftLimit(SoftLimitDirection.kForward);
        return telescopeEncoder.getPosition() / maxPosition;
    }

    public void telescopeArm(double input) {
        double inputScaled = MathUtil.applyDeadband(input, 0.05) * ArmConstants.LIFT_INPUT_SCALAR;
        double inputRPM = inputScaled * Constants.NEO_MAX_RPM;
        telescopeArmRaw(inputRPM);
    }

    /**
     * DO NOT USE FOR JOYSTICK INPUT.
     * 
     * @param rpm
     */
    public void telescopeArmRaw(double rpm) {
    }

    /**
     * Hold the arm telescope at the current position. <br>
     * </br>
     * This gets the current encoder position and sets in the SparkMaxPID with the
     * Position control type.
     */
    public void holdTelescopePosition() {
    }

    /**
     * Home the arm telescope. This is used for setting soft limits.
     * 
     * @param retracted If true, set the soft-limit for the telescope fully
     *                  retracted. If false, set the soft limit for the telescope
     *                  fully extended.
     */
    public void homeTelescope(boolean retracted) {
        if (retracted) {
            telescopeEncoder.setPosition(0);
            telescopeArm.setSoftLimit(SoftLimitDirection.kReverse, (float) telescopeEncoder.getPosition());
        } else if (!retracted) {
            telescopeArm.setSoftLimit(SoftLimitDirection.kForward, (float) telescopeEncoder.getPosition());
        }
    }

    /**
     * Check if the brake is on the arm
     * 
     * @return whether the brake is applied on the arm
     */
    // public boolean getBrakeApplied() {
    // return rotateArmBrake.get() == DoubleSolenoid.Value.kForward;
    // }

    // ----------------- PRIVATE HELPER METHODS ----------------- //

    // private void applyBrake() {
    // double currentVel = rotateEncoder.getVelocity();

    // if (Math.abs(currentVel) < 0.01) {
    // rotateArmBrake.set(DoubleSolenoid.Value.kForward);
    // rotateArm.setIdleMode(IdleMode.kBrake);
    // }
    // }

    // private void releaseBrake() {
    // rotateArm.setIdleMode(IdleMode.kBrake);
    // rotateArmBrake.set(DoubleSolenoid.Value.kReverse);
    // }
}
