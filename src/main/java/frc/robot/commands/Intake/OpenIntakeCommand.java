package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class OpenIntakeCommand extends InstantCommand {
    public OpenIntakeCommand(){
        super(()-> IntakeSubsystem.getInstance().openIntake(), IntakeSubsystem.getInstance());
    }
}
