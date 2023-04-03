// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.experimental;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.LimelightSubsytem;

public class PointToTagCommand extends CommandBase {
  private final LimelightSubsytem limelightSubsytem;
  private final DiffDriveSubsystem diffDriveSubsystem;
  private final Set<Subsystem> subsystems;
  private final PIDController pid;
  /** Creates a new PointToTagCommand. */
  public PointToTagCommand(LimelightSubsytem limelightSubsytem, DiffDriveSubsystem diffDriveSubsystem)
  {
    this.diffDriveSubsystem = diffDriveSubsystem;
    this.limelightSubsytem = limelightSubsytem;
    this.subsystems = Set.of(this.diffDriveSubsystem, this.limelightSubsytem);
    pid = new PIDController(0.002, 0.08, 0.001);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    diffDriveSubsystem.arcadeDrive(0, -pid.calculate(limelightSubsytem.getTx(),0));
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
