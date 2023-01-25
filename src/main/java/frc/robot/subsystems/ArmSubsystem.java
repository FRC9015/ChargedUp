package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;



public class ArmSubsystem implements Subsystem
{
    private final CANSparkMax rotateArm; //rotateArm pivots the arm. 
    private final CANSparkMax telescopeArm; //telescopeArm moves the arm in and out.

    // Creates a new ArmSubsystem.
    public ArmSubsystem() 
    {
        rotateArm = new CANSparkMax(ArmConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
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