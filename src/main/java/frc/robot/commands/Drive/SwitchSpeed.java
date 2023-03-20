// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;

public class SwitchSpeed extends InstantCommand {

  public SwitchSpeed() {
    super(RobotState::toggleSlow);
  }
}
