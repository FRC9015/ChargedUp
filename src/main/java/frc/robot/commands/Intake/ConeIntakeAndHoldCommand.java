package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakePneumaticSubsystem;

public class ConeIntakeAndHoldCommand extends CommandBase {
    private IntakePneumaticSubsystem intake = IntakePneumaticSubsystem.getInstance();
    private Timer timer;

    public ConeIntakeAndHoldCommand() {
        addRequirements(intake);
        timer = new Timer();
        timer.reset();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        intake.closeIntake();
        intake.setIntakeMotorSpeed(0.45);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end if running for longer than 1.5s
        return timer.get() > 1.5;
    }
}
