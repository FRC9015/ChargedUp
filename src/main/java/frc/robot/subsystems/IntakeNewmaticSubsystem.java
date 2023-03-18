package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeNewmaticSubsystem implements Subsystem{

    private final static IntakeNewmaticSubsystem INSTANCE = new IntakeNewmaticSubsystem();
    private boolean forward;
    @SuppressWarnings("WeakerAccess")
    public static IntakeNewmaticSubsystem getInstance() {
        return INSTANCE;
        
    }


    CANSparkMax intakeMotor;
    DoubleSolenoid intake;
    //PneumaticHub pHub;

    private IntakeNewmaticSubsystem(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(20, 30);
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 13, 12);
        forward = false;
    }

    public void openIntake(){
        intake.set(DoubleSolenoid.Value.kForward);
    }

    public void closeIntake(){
        intake.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void setIntakeMotorSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void switchIntake(){
        if(forward){       
             intake.set(DoubleSolenoid.Value.kReverse);
             forward = false;

        }else{
            intake.set(DoubleSolenoid.Value.kForward);
            forward = true;
        }
    }


}
