package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final DoubleSolenoid rotateArmBrake;

    private final CANSparkMax telescopeArm; // telescopeArm moves the arm in and out.
    private final RelativeEncoder telescopeEncoder;
    private final SparkMaxPIDController telescopePID;
    private final PIDFConstants telescopePIDConstants;

    private ArmSubsystem() 
    {
        // Set up stuff for the Arm rotating motors
        rotateArm = new CANSparkMax(ArmConstants.ARM_LIFT_CAN_ID, MotorType.kBrushless);

        rotateEncoder = rotateArm.getEncoder();
        rotatePID = rotateArm.getPIDController();

        rotatePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1); // TODO: THESE CONSTANTS MUST BE TUNED
        rotatePIDConstants.updateSparkMax(rotatePID);

        // Initialize arm rotation brake and make sure it is released on boot
        rotateArmBrake = PneumaticHubSubsystem.getDoubleSolenoid(ArmConstants.ARM_BRAKE_SOLENOID);
        rotateArmBrake.set(DoubleSolenoid.Value.kReverse);

        // Set up stuff for the Arm Telescoping motor
        telescopeArm = new CANSparkMax(ArmConstants.ARM_TELESCOPE_CAN_ID, MotorType.kBrushless);

        telescopeEncoder = telescopeArm.getEncoder();
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

    public void rotateArm(double motorspeed){
        rotateArm.set(motorspeed);
    }

    public void telescopeArm(double motorspeed){
        telescopeArm.set(motorspeed);
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
