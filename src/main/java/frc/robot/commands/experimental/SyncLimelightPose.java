// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.experimental;

import java.util.Set;

import javax.swing.filechooser.FileFilter;

import edu.wpi.first.hal.simulation.SimDeviceDataJNI.SimDeviceInfo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.LimelightSubsytem;

public class SyncLimelightPose extends CommandBase {
  private final LimelightSubsytem limelightSubsytem;
  private final DiffDriveSubsystem diffDriveSubsystem;
  private final Set<Subsystem> subsystems;
  private boolean finished;
  /** Creates a new PointToTagCommand. */
  public SyncLimelightPose(LimelightSubsytem limelightSubsytem, DiffDriveSubsystem diffDriveSubsystem)
  {
    this.diffDriveSubsystem = diffDriveSubsystem;
    this.limelightSubsytem = limelightSubsytem;
    this.subsystems = Set.of(this.limelightSubsytem,this.diffDriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelightSubsytem.hasTargets()){
    //diffDriveSubsystem.resetOdometry(new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5]*(3.1415/180))));
    diffDriveSubsystem.resetOdometry(limelightSubsytem.getLimelightPose());
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public Set<Subsystem> getRequirements() {
      return this.subsystems;
  }
}
