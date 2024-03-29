package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

public class ArmUpCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    public ArmUpCommand() {
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.rotateArm(0.2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.rotateArm(0);
    }
}
