package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class CloseIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public CloseIntakeCommand(){
        addRequirements(intakeSubsystem);
    }

    public void initialize(){
        intakeSubsystem.closeIntake();
    }
}
