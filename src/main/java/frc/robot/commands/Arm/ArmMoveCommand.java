// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import lombok.Setter;

public class ArmMoveCommand extends CommandBase {
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  @Setter
  private double armLiftSpeed, armTelescopeSpeed;

  public ArmMoveCommand(double liftSpeed, double telescopeSpeed) {
    addRequirements(arm);

    armLiftSpeed = liftSpeed;
    armTelescopeSpeed = telescopeSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.rotateArm(armLiftSpeed);
    arm.telescopeArm(armTelescopeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.rotateArm(0);
    arm.telescopeArm(0);
    arm.holdRotatePosition();
    arm.holdTelescopePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
