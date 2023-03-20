package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class CloseIntakeCommand extends InstantCommand {
    public CloseIntakeCommand(){
        super(()-> IntakeSubsystem.getInstance().closeIntake(), IntakeSubsystem.getInstance());
    }
}
