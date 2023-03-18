// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticHubSubsystem;

/**
 * Command that finishes only once the pneumatics system is pressurized.
 */
public class WaitForPressureCommand extends CommandBase {
  PneumaticHubSubsystem pHub = PneumaticHubSubsystem.getInstance();
  
  public WaitForPressureCommand() {
    addRequirements(pHub);
  }

  @Override
  public void initialize() {
    pHub.enableCompressor();
  }

  @Override
  public boolean isFinished() {
    return pHub.isPressurized();
  }
}
