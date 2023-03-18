package frc.robot.subsystems;

import java.io.PipedInputStream;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;



public class ArmSubsystem implements Subsystem
{
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }


    private final CANSparkMax rotateArm; //rotateArm pivots the arm. 
    private final CANSparkMax telescopeArm; //telescopeArm moves the arm in and out.
    private RelativeEncoder rotateEncoder, telescopeEncoder;
    private PIDController pid;
    private double kP = 0.02;
    private double kI = 0;
    private double kD =0;
    private boolean activatePID;

    private final DoubleSolenoid rotateArmBrake;


    private double rotatePidSetpoint, telescopePidSetpoint;

    // Creates a new ArmSubsystem.
    private ArmSubsystem() 
    {
        
        pid = new PIDController(kP, kI, kD);
        rotateArm = new CANSparkMax(ArmConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        rotateArm.setIdleMode(IdleMode.kBrake);
        telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
        telescopeArm.setIdleMode(IdleMode.kBrake);

        rotateEncoder = rotateArm.getEncoder();
        rotatePidSetpoint = rotateEncoder.getPosition();
        
        telescopeEncoder = telescopeArm.getEncoder();
        telescopePidSetpoint = telescopeEncoder.getPosition();

        rotateEncoder.setPosition(0);
        telescopeEncoder.setPosition(0);

        activatePID = false;


        rotateArmBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15,14);
        rotateArmBrake.set(DoubleSolenoid.Value.kForward);
    }

    public void rotateArm(double motorspeed){
        if( motorspeed!=0){
        releaseBrake();
        rotateArm.set(motorspeed);
        }
    }

    public void telescopeArm(double motorspeed){
        telescopeArm.set(motorspeed);
    }

    public void setRotatePid(double setpoint){
        rotatePidSetpoint = setpoint;
    }

    public void SetActivatePID(boolean active){
        System.out.println("setarmy");
        activatePID = active;
    }

    public double getRotEncoderPos(){
        return rotateEncoder.getPosition();}
    
    public double getTeleEncoderPos(){
        return telescopeEncoder.getPosition();
    }
    

    public void setPID(){
        System.out.print(rotateEncoder.getPosition());
        System.out.println(rotatePidSetpoint);
        System.out.println(activatePID);
        if( activatePID){
            releaseBrake();
            rotateArm.set(pid.calculate(rotateEncoder.getPosition(), rotatePidSetpoint));
        }
    }

    
    public void periodic()
    {
        applyBrake();
    }
    
    
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

        /**
     * Check if the brake is on the arm
     * @return whether the brake is applied on the arm
     */
    public boolean getBrakeApplied () {
        return rotateArmBrake.get() == DoubleSolenoid.Value.kReverse;
    }


    // ----------------- PRIVATE HELPER METHODS ----------------- //

    private void applyBrake() {
        double currentVel = rotateArm.getAppliedOutput();
 

        if (Math.abs(currentVel) == 0) {
            rotateArmBrake.set(DoubleSolenoid.Value.kReverse);
        }
    }

    private void releaseBrake() {
        rotateArmBrake.set(DoubleSolenoid.Value.kForward);
    }
}
