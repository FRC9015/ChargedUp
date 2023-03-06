// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class GoToPointCommand extends CommandBase {
  private final LimelightSubsystem limelightSubsytem;
  //private final DiffDriveSubsystem diffDriveSubsystem;
  private final Set<Subsystem> subsystems;
  /** Creates a new PointToTagCommand. */
  public GoToPointCommand(LimelightSubsystem limelightSubsytem)
  {
    //this.diffDriveSubsystem = diffDriveSubsystem;
    this.limelightSubsytem = limelightSubsytem;
    this.subsystems = Set.of(this.limelightSubsytem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] botpose = limelightSubsytem.getBotpose();
    System.out.print(botpose[0]);
    System.out.print(botpose[1]);
    System.out.print(botpose[2]);
    System.out.print(botpose[3]);
    System.out.print(botpose[4]);
    System.out.print(botpose[5]);

    
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
