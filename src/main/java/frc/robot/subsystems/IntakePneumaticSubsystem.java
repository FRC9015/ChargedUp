package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;

public class IntakePneumaticSubsystem extends SubsystemBase{

    private static IntakePneumaticSubsystem INSTANCE;
    
    @SuppressWarnings("WeakerAccess")
    public static IntakePneumaticSubsystem getInstance() {
        if(INSTANCE == null) INSTANCE = new IntakePneumaticSubsystem();
        return INSTANCE;
        
    }

    private boolean forward;

    CANSparkMax intakeMotor;
    DoubleSolenoid intake;

    private IntakePneumaticSubsystem(){
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_DRIVE_CAN_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(20, 30);
        intake = PneumaticHubSubsystem.getDoubleSolenoid(IntakeConstants.OPEN_CLOSE_SOLENOID);
        forward = false;
    }

    @Override
    public void periodic() {
        System.out.println(intake.get()==DoubleSolenoid.Value.kForward);

        RobotState.setIntakeOpen(!(intake.get()==DoubleSolenoid.Value.kReverse));
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
