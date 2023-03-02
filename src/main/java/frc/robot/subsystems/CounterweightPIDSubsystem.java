package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.CounterweightConstants;

public class CounterweightPIDSubsystem extends PIDSubsystem {
    
    private final static CounterweightPIDSubsystem INSTANCE = new CounterweightPIDSubsystem();
    DigitalInput limit = new DigitalInput(0);

    private TalonSRX counterWeightMotor;

    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    private boolean activatePID = false;

    public static CounterweightPIDSubsystem getInstance(){
        return INSTANCE;
    }
    

    private CounterweightPIDSubsystem(){
        super(new PIDController(kP, kI, kD));

        counterWeightMotor = new TalonSRX(CounterweightConstants.DRIVE_MOTOR_CAN_ID);
        counterWeightMotor.getSelectedSensorPosition();
        counterWeightMotor.set(TalonSRXControlMode.PercentOutput,0);
        counterWeightMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public boolean getSwitch(){
        return limit.get();
    }

    public void setMotor(double speed){
        counterWeightMotor.set(TalonSRXControlMode.PercentOutput, speed);
        
    }
    public void setEncoder(double encoderValue){
        counterWeightMotor.setSelectedSensorPosition(encoderValue);
    }

    public double getEncoder(){
        return counterWeightMotor.getSelectedSensorPosition();
    }

    public void setPIDsetPoint(double setpoint){
        getController().setSetpoint(setpoint);
    }
    public void periodic(){
        //System.out.println(counterWeightMotor.getSelectedSensorPosition());
        if (activatePID){
            //getController().calculate()
        }
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }
}
