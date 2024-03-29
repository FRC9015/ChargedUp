package frc.robot.commands.Arm;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.ArmSubsystem;

public class ArmInCommand implements Command {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final Set<Subsystem> subsystems;

    public ArmInCommand() {
        this.subsystems = Set.of(this.armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.telescopeArm(-0.95);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.telescopeArm(0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
