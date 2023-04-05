package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem.LEDPreset;
import frc.robot.subsystems.LEDSubsystem.LEDeffect;

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


    @Override 
    public void periodic() {
        RobotState.setFeetDown(!(foot.get() == Value.kReverse));
        
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
             LEDSubsystem.getInstance().setPreset(LEDPreset.LOGOSLOW);

        }else{
            foot.set(DoubleSolenoid.Value.kForward);
            forward = true;
            LEDSubsystem.getInstance().setPreset(LEDPreset.RAINBOW);


        }
    }


}
