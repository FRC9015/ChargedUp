package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;
public class ArmSubsystem implements Subsystem
{
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }


    private final CANSparkMax rotateArm, rotateArmSecond; //rotateArm pivots the arm. 
    private final CANSparkMax telescopeArm; //telescopeArm moves the arm in and out.

    private final DoubleSolenoid rotateArmBrake;

    // Creates a new ArmSubsystem.
    private ArmSubsystem() 
    {
        rotateArm = new CANSparkMax(IntakeConstants.ARM_LIFT_CAN_ID, MotorType.kBrushless);
        rotateArmSecond = new CANSparkMax(IntakeConstants.ARM_LIFT_2_CAN_ID, MotorType.kBrushless);

        rotateArmSecond.follow(rotateArm);

        rotateArmBrake = PneumaticHubSubsystem.getInstance().getDoubleSolenoid(IntakeConstants.ARM_BRAKE_SOLENOID);

        telescopeArm = new CANSparkMax(IntakeConstants.ARM_TELESCOPE_CAN_ID, MotorType.kBrushless);
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
