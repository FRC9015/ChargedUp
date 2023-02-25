// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armpidCommand extends CommandBase {
  

  private PIDController myPidController;
  private final ArmSubsystem armSubsystem;
  private final Set<Subsystem> subsystems;

  /** Creates a new armpid. */
  public armpidCommand(ArmSubsystem myArmSubsystem,double setpoint) {
    myPidController = new PIDController(0.02, 0, 0);
    myPidController.setSetpoint(setpoint);
    myPidController.setTolerance(1);
        
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.armSubsystem = myArmSubsystem;
    this.subsystems = Set.of(this.armSubsystem);
    
  }
  public void execute(){
    armSubsystem.rotateArm( myPidController.calculate(armSubsystem.getRotEncoderPos()));
  }

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
