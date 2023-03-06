package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.PIDFConstants;
public class ArmSubsystem implements Subsystem
{
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }


    private final CANSparkMax rotateArm, rotateArmSecond; // rotateArm pivots the arm. 
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
        rotateArm = new CANSparkMax(IntakeConstants.ARM_LIFT_CAN_ID, MotorType.kBrushless);
        rotateArmSecond = new CANSparkMax(IntakeConstants.ARM_LIFT_2_CAN_ID, MotorType.kBrushless);

        rotateArmSecond.follow(rotateArm);

        rotateEncoder = rotateArm.getEncoder();
        rotatePID = rotateArm.getPIDController();

        rotatePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1);
        rotatePIDConstants.updateSparkMax(rotatePID);

        rotateArmBrake = PneumaticHubSubsystem.getInstance().getDoubleSolenoid(IntakeConstants.ARM_BRAKE_SOLENOID);
        rotateArmBrake.set(DoubleSolenoid.Value.kReverse);

        telescopeArm = new CANSparkMax(IntakeConstants.ARM_TELESCOPE_CAN_ID, MotorType.kBrushless);

        telescopeEncoder = telescopeArm.getEncoder();
        telescopePID = telescopeArm.getPIDController();

        telescopePIDConstants = new PIDFConstants(0.1, 0, 0 , 0, 0.1);
        telescopePIDConstants.updateSparkMax(telescopePID);
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

    public void periodic()
    {
        // This method will be called once per scheduler run
    }
    
    
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
