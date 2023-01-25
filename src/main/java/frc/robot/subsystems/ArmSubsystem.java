package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;



public class ArmSubsystem implements Subsystem
{
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }


    private final CANSparkMax rotateArm; //rotateArm pivots the arm. 
    // private final CANSparkMax telescopeArm; //telescopeArm moves the arm in and out.

    // Creates a new ArmSubsystem.
    private ArmSubsystem() 
    {
        rotateArm = new CANSparkMax(ArmConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        //telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
    }

    public void moveArm(double motorspeed){
        rotateArm.set(motorspeed);
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
