// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOpenWhileActive extends StartEndCommand {
  public IntakeOpenWhileActive() {
    super(() -> IntakeSubsystem.getInstance().openIntake(), () -> IntakeSubsystem.getInstance().closeIntake(),
        IntakeSubsystem.getInstance());
  }
}
