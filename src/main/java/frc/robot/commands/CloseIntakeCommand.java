package frc.robot.commands;

import frc.robot.subsystems.IntakeNewmaticSubsystem;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class CloseIntakeCommand implements Command{
    private final IntakeNewmaticSubsystem intakeNewmaticSubsystem;
    private final Set<Subsystem> subsystems;

    public CloseIntakeCommand(IntakeNewmaticSubsystem intakeNewmaticSubsystem){
        this.intakeNewmaticSubsystem = intakeNewmaticSubsystem;
        this.subsystems = Set.of(this.intakeNewmaticSubsystem);

    }

    public void initialize(){
        intakeNewmaticSubsystem.closeIntake();
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
