package frc.robot.commands.Intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.IntakePneumaticSubsystem;

public class CloseIntakeCommand implements Command {
    private final IntakePneumaticSubsystem intakeNewmaticSubsystem =
            IntakePneumaticSubsystem.getInstance();
    private final Set<Subsystem> subsystems;

    public CloseIntakeCommand() {
        this.subsystems = Set.of(this.intakeNewmaticSubsystem);
    }

    public void initialize() {
        intakeNewmaticSubsystem.closeIntake();
    }

    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return this.subsystems;
    }
}
