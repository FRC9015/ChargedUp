package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeNewmaticSubsystem implements Subsystem{

    private final static IntakeNewmaticSubsystem INSTANCE = new IntakeNewmaticSubsystem();
    
    @SuppressWarnings("WeakerAccess")
    public static IntakeNewmaticSubsystem getInstance() {
        return INSTANCE;
    }



    DoubleSolenoid intake;
    PneumaticHub pHub;

    private IntakeNewmaticSubsystem(){
        //pHub = new PneumaticHub();
        intake = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    }

    public void openIntake(){
        intake.set(DoubleSolenoid.Value.kForward);
    }

    public void closeIntake(){
        intake.set(DoubleSolenoid.Value.kReverse);
    }




}
