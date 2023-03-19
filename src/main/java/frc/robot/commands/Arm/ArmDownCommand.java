package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDownCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    public ArmDownCommand() {
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.rotateArm(0.5);
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
