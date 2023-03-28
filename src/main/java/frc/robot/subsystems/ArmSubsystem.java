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
        System.out.print("torque");
        System.out.println(getArmTorque());
        rotateArm.set((motorspeed*0.5)+ getArmTorque()*0.0025);
        // if (motorspeed != 0) {
        //     rotateArmBrake.set(DoubleSolenoid.Value.kForward);
        //     //System.out.println(motorspeed);
            
        //         rotateArm.set(motorspeed*0.5);
            
        //         rotateArm.set(motorspeed*0.5);
            
        // } else {
        //     rotateArm.set(0);
        //     //(DoubleSolenoid.Value.kReverse);

        // }
    }

    public void telescopeArm(double motorspeed) {
        telescopeArm.set(motorspeed*0.95);
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
        final double leverarm = ArmConstants.stageOneLengthMeters+ArmConstants.stageTwoLengthMeters*(getTeleEncoderPos()/0.62);
        final double theta = ArmConstants.armMinRotAngle+(getRotEncoderPos()/3.73)*(ArmConstants.armMaxRotAngle-ArmConstants.armMinRotAngle);

        final double torque = leverarm*ArmConstants.armForceNewtons*Math.sin(Math.toRadians(theta));

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
