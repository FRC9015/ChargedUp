// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends InstantCommand {
  public ToggleIntakeCommand() {
    super(()-> IntakeSubsystem.getInstance().switchIntake(), IntakeSubsystem.getInstance());
  }
}
