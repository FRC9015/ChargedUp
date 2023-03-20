package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.CounterweightConstants;

public class CounterweightPIDSubsystem extends PIDSubsystem {
    
    private final static CounterweightPIDSubsystem INSTANCE = new CounterweightPIDSubsystem();
    DigitalInput limit = new DigitalInput(CounterweightConstants.ENDSTOP_SWITCH_PORT);

    private CANSparkMax counterWeightMotor;
    private RelativeEncoder counterWeightEncoder;

    private static double kP = 0.1;
    private static double kI = 0;
    private static double kD = 0;

    private boolean activatePID = false;

    public static CounterweightPIDSubsystem getInstance(){
        return INSTANCE;
    }
    

    private CounterweightPIDSubsystem(){
        super(new PIDController(kP, kI, kD));

        counterWeightMotor = new CANSparkMax(CounterweightConstants.DRIVE_MOTOR_CAN_ID, MotorType.kBrushless);
        counterWeightEncoder = counterWeightMotor.getEncoder();

    }
    
    public boolean getSwitch(){
        return limit.get();
    }

    public double getEncoder(){
        return counterWeightEncoder.getPosition();
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
