package frc.robot.subsystems;

import java.io.PipedInputStream;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
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
    private final TalonSRX telescopeArm; //telescopeArm moves the arm in and out.
    private RelativeEncoder rotateEncoder;
    private PIDController pid;
    private double kP = 0.02;
    private double kI = 0;
    private double kD =0;
    private boolean activatePID;

    private double pidSetpoint;

    // Creates a new ArmSubsystem.
    private ArmSubsystem() 
    {
        
        pid = new PIDController(kP, kI, kD);
        rotateArm = new CANSparkMax(ArmConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        rotateArm.setIdleMode(IdleMode.kBrake);
        telescopeArm = new TalonSRX(ArmConstants.TELESCOPE_CAN_ID);
        rotateEncoder = rotateArm.getEncoder();
        pidSetpoint = rotateEncoder.getPosition();
        activatePID = false;
    }

    public void rotateArm(double motorspeed){
        rotateArm.set(motorspeed);
    }

    public void telescopeArm(double motorspeed){
        telescopeArm.set(TalonSRXControlMode.PercentOutput,motorspeed);
    }

    public void setRotatePid(double setpoint){
        pidSetpoint = setpoint;
    }

    public void SetActivatePID(boolean active){
        System.out.println("setarmy");
        activatePID = active;
    }

    public double getRotEncoderPos(){
        return rotateEncoder.getPosition();
    }

    

    public void setPID(){
        System.out.print(rotateEncoder.getPosition());
        System.out.println(pidSetpoint);
        System.out.println(activatePID);
        if( activatePID){
            System.out.println("pid should go");
            rotateArm.set(pid.calculate(rotateEncoder.getPosition(), pidSetpoint));
        }
    }

    
    public void periodic()
    {
        System.out.println(rotateEncoder.getPosition());
    }
    
    
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }
}
