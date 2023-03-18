package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class FootSubsystem implements Subsystem{

    private final static FootSubsystem INSTANCE = new FootSubsystem();
    private boolean forward;
    @SuppressWarnings("WeakerAccess")
    public static FootSubsystem getInstance() {
        return INSTANCE;
        
    }


    DoubleSolenoid foot;
    //PneumaticHub pHub;

    private FootSubsystem(){
       
        foot = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 10);
     }

    public void footDown(){
        foot.set(DoubleSolenoid.Value.kForward);
    }

    public void footUp(){
        foot.set(DoubleSolenoid.Value.kReverse);
    }
    


    public void toggleFoot(){
        if(forward){       
             foot.set(DoubleSolenoid.Value.kReverse);
             forward = false;

        }else{
            foot.set(DoubleSolenoid.Value.kForward);
            forward = true;
        }
    }


}
