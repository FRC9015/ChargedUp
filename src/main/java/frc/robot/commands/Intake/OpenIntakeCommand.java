package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeNewmaticSubsystem;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class OpenIntakeCommand implements Command{
    private final IntakeNewmaticSubsystem intakeNewmaticSubsystem;
    private final Set<Subsystem> subsystems;

    public OpenIntakeCommand(IntakeNewmaticSubsystem intakeNewmaticSubsystem){
        this.intakeNewmaticSubsystem = intakeNewmaticSubsystem;
        this.subsystems = Set.of(this.intakeNewmaticSubsystem);

    }

    public void initialize(){
        intakeNewmaticSubsystem.openIntake();
    }
    public boolean isFinished(){
        return true;
    }


    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return this.subsystems;
    }

}
