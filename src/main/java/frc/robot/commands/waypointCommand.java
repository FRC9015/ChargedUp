// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

//import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
import frc.robot.RobotState;


public class waypointCommand extends CommandBase {
  private final LimelightSubsytem limelightSubsytem;
  private final DiffDriveSubsystem diffDriveSubsystem;
  private final Set<Subsystem> subsystems;
  /** Creates a new waypointCommand. */
  public waypointCommand(LimelightSubsytem limelightSubsytem, DiffDriveSubsystem diffDriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.diffDriveSubsystem = diffDriveSubsystem;
    this.limelightSubsytem = limelightSubsytem;
    this.subsystems = Set.of(this.limelightSubsytem,this.diffDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
      RobotState.setSavedPoint(diffDriveSubsystem.getPose());
      

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

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
