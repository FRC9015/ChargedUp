package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.Constants.PneumaticConstants;

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
       
        foot = PneumaticHubSubsystem.getDoubleSolenoid(PneumaticConstants.LIFT_FEET_CONSTANTS);
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

        }else{
            foot.set(DoubleSolenoid.Value.kForward);
            forward = true;
        }
    }


}
