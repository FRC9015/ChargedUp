// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.DigestInputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DiffDriveSubsystem;

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
