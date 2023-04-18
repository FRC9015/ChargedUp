// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class RavisRamseteCommand extends CommandBase {
  private final DiffDriveSubsystem diffDriveSubsystem;
  private final Set<Subsystem> subsystems;
  


  public RavisRamseteCommand(DiffDriveSubsystem diffDriveSubsystem) {
    this.diffDriveSubsystem = diffDriveSubsystem;
    this.subsystems = Set.of(this.diffDriveSubsystem);
    
  }

  
    
       
 

  /** Creates a new ramseteCommand. 
   * 
   *
   * @return */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diffDriveSubsystem.runRamseteCommand( frc.robot.RobotState.getSavedPoint(), diffDriveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return this.subsystems;
  }

}
