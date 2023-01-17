// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PigeonSubsystem;

/** 
 * Simple Command that Resets the Pigeon's Angles
 */
public class ResetAngleCommand extends InstantCommand {

  final PigeonSubsystem pigeon = PigeonSubsystem.getInstance();

  /** Creates a new ResetAngleCommand. */
  public ResetAngleCommand() {
    addRequirements(pigeon);
  }

  @Override
  public void initialize() {
    pigeon.resetAngles();
  }
}
