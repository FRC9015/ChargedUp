// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.PIDFConstants;

public class UpdatePIDFConstantsCommand extends InstantCommand {

  private final PIDFConstants constants;
  private final SparkMaxPIDController[] controllers;
  /**
   * 
   * @param constants Constants object to use for updating the controllers
   * @param controllers The controllers to update
   */
  public UpdatePIDFConstantsCommand(PIDFConstants constants, SparkMaxPIDController... controllers) {
    this.constants = constants;
    this.controllers = controllers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(SparkMaxPIDController controller : controllers) {
      constants.updateSparkMax(controller);
    }
  }
}
