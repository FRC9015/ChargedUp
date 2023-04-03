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

public class ArmSubsystem implements Subsystem {
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    //arm straight up encoder: 1.81
    //telescope out: 0.62

    //high cube:rot:0.758 tele:0.595


    private final CANSparkMax rotateArm; // rotateArm pivots the arm.
    private final double kStartingArmPosition;
    private final double kMaxArmPosition = 1; // TODO CHANGE THIS
    private final CANSparkMax telescopeArm; // telescopeArm moves the arm in and out.
    private final double kStartingTelescopePosition;
    private RelativeEncoder rotateEncoder, telescopeEncoder;
    private PIDController pid;

    private double kP = 0.02;
    private double kI = 0;
    private double kD = 0;
    private boolean activatePID;

    private final DoubleSolenoid rotateArmBrake;

    private double torque;

    private double rotatePidSetpoint, telescopePidSetpoint;


    private double rotOffset, teleOffset;

    // Creates a new ArmSubsystem.
    private ArmSubsystem() {

        pid = new PIDController(kP, kI, kD);
        rotateArm = new CANSparkMax(ArmConstants.ROTATE_CAN_ID, MotorType.kBrushless);
        rotateArm.setIdleMode(IdleMode.kBrake);
        telescopeArm = new CANSparkMax(ArmConstants.TELESCOPE_CAN_ID, MotorType.kBrushless);
        telescopeArm.setIdleMode(IdleMode.kBrake);

        rotateEncoder = rotateArm.getEncoder();
        kStartingArmPosition = rotateEncoder.getPosition();
        rotatePidSetpoint = rotateEncoder.getPosition();

        telescopeEncoder = telescopeArm.getEncoder();
        kStartingTelescopePosition = telescopeEncoder.getPosition();
        telescopePidSetpoint = telescopeEncoder.getPosition();

        rotateEncoder.setPosition(0);
        telescopeEncoder.setPosition(0);

        activatePID = false;

        rotateArmBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 14);
        rotateArmBrake.set(DoubleSolenoid.Value.kReverse);

        rotOffset =0;
        teleOffset=0;

        torque = 0;
    }

    public void changeTeleOffset(double change){
        teleOffset+=change;
    }

    public void changeRotOffset(double change){
        rotOffset+=change;
    }

    public void resetArm(){
        rotateEncoder.setPosition(0);
        telescopeEncoder.setPosition(0);
    }

    public void rotateArm(double motorspeed) {


        if (getRotEncoderPos()<=0.03){
            rotateArm.set(Math.max(-0.2, motorspeed*0.95));

        }else if(getRotEncoderPos()>=3.7){
            rotateArm.set(Math.min(0.2, motorspeed*0.95));
        }
        else{
        rotateArm.set(motorspeed);
        }



    }

    public void telescopeArm(double motorspeed) {
        if (getTeleEncoderPos()<=0.03){
            telescopeArm.set(Math.max(-0.2, motorspeed*0.95));

        }else if(getTeleEncoderPos()>=0.6){
            telescopeArm.set(Math.min(0.2, motorspeed*0.95));
        }
        else{
        telescopeArm.set(motorspeed);
        }
    }

    private boolean armSafeToRaise() {
        boolean armAboveLimit = Math.abs(rotateEncoder.getPosition() - kStartingArmPosition) < 0.05;
        boolean armTelescopeRetracted = Math.abs(telescopeEncoder.getPosition() - kStartingTelescopePosition) < 0.05;

        return armAboveLimit ? armTelescopeRetracted : true;
    }

    public void setRotatePid(double setpoint) {
        rotatePidSetpoint = setpoint;
    }

    public void SetActivatePID(boolean active) {
        System.out.println("setarmy");
        activatePID = active;
    }

    

    public double getRotEncoderPos() {
        return rotateEncoder.getPosition()+rotOffset;
    }

    public void lockarm(){
        rotateArm.set(0.1);
    }

    public double getTeleEncoderPos() {
        return telescopeEncoder.getPosition()+teleOffset;
    }

    public void setPID() {
        System.out.print(rotateEncoder.getPosition());
        System.out.println(rotatePidSetpoint);
        System.out.println(activatePID);
        if (activatePID) {
            releaseBrake();
            rotateArm.set(pid.calculate(rotateEncoder.getPosition(), rotatePidSetpoint));
        }
    }

    public double getArmTorque(){

        // Calculate lever arm using lengths of arm's two stages and position of telescoping encoder
        final double leverarm = ArmConstants.STAGE_ONE_LENGTH_METERS+ArmConstants.STAGE_TWO_LENGTH_METERS*(getTeleEncoderPos()/0.62);
    
        // Calculate rotation angle using positions of rotational encoder and min/max rotation angles
        final double theta = ArmConstants.ARM_MIN_ROTATE_ANGLE+(getRotEncoderPos()/3.73)*(ArmConstants.ARM_MAX_ROTATE_ANGLE-ArmConstants.ARM_MIN_ROTATE_ANGLE);
    
        // Calculate torque using lever arm, constant arm force, and sine of rotation angle in radians
        final double torque = leverarm*ArmConstants.ARM_FORCE_NEWTONS*Math.sin(Math.toRadians(theta));
    
        // Return calculated torque
        return torque;
    }
    
    public void periodic() {


        // System.out.print("telescope position:");
        // System.out.println(telescopeEncoder.getPosition());
        //applyBrake();
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Check if the brake is on the arm
     * 
     * @return whether the brake is applied on the arm
     */
    public boolean getBrakeApplied() {
        return rotateArmBrake.get() == DoubleSolenoid.Value.kReverse;
    }

    // ----------------- PRIVATE HELPER METHODS ----------------- //

    private void applyBrake() {
        double currentVel = rotateArm.getAppliedOutput();

        if (Math.abs(currentVel) == 0) {
            //rotateArmBrake.set(DoubleSolenoid.Value.kReverse);
        }else{
            //rotateArmBrake.set(DoubleSolenoid.Value.kForward);
        }
    }

    private void releaseBrake() {
        //rotateArmBrake.set(DoubleSolenoid.Value.kForward);
    }
}
