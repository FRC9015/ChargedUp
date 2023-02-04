// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CounterweightPIDSubsystem;

public class WeightForwardCommand extends CommandBase {
  private CounterweightPIDSubsystem counterweightPIDSubsystem;

  /** Creates a new WeightForwardCommand. */
  public WeightForwardCommand(CounterweightPIDSubsystem newcounterweightPIDSubsystem) {
    counterweightPIDSubsystem = newcounterweightPIDSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counterweightPIDSubsystem.setMotor(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    counterweightPIDSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
