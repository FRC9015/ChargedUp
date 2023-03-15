// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.ModuleLayer.Controller;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ArmSubsystem;

public class armDefualtControlCommand extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final Set<Subsystem> subsystems;


  private OperatorController controller;
  /** Creates a new armDefualtControlCommand. */
  public armDefualtControlCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.subsystems = Set.of(this.armSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(controller.getTankLeft())>0.05){
    armSubsystem.rotateArm(controller.getTankLeft());}


    if (Math.abs(controller.getTankRight())>0.05){
    armSubsystem.telescopeArm(controller.getTankRight());}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return this.subsystems;
  }
}
