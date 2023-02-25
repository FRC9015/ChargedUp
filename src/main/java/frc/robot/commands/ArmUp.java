package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

import java.util.Set;

public class ArmUp implements Command {
    private final ArmSubsystem armSubsystem;
    private final Set<Subsystem> subsystems;

    public ArmUp(ArmSubsystem myArmSubsystem) {
        this.armSubsystem = myArmSubsystem;
        this.subsystems = Set.of(this.armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.rotateArm(0.5);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.rotateArm(0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
