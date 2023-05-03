package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

import java.util.Set;

public class ArmOutCommand implements Command {
    private final ArmSubsystem armSubsystem;
    private final Set<Subsystem> subsystems;

    public ArmOutCommand(ArmSubsystem myArmSubsystem) {
        this.armSubsystem = myArmSubsystem;
        this.subsystems = Set.of(this.armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.telescopeArm(0.95);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
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
