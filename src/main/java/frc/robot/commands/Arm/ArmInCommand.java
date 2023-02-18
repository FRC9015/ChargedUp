package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmInCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    public ArmInCommand() {
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.telescopeArm(-0.95);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.telescopeArm(0);
    }
}
