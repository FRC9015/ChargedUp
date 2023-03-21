// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeWheelsCommand extends CommandBase {
  public static enum IntakeDriveDirection {
    In,
    Out
  }

  IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  double speed = 0;

  public RunIntakeWheelsCommand(IntakeDriveDirection direction) {
    addRequirements(intakeSubsystem);
    switch (direction) {
      case In:
        speed = 0.5;
      case Out:
        speed = -0.5;
    }
  }

  public RunIntakeWheelsCommand(double driveSpeed) {
    addRequirements(intakeSubsystem);
    speed = driveSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runIntakeDrive(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
