package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class OpenIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public OpenIntakeCommand(){
        addRequirements(intakeSubsystem);
    }

    public void initialize(){
        intakeSubsystem.openIntake();
    }
}
