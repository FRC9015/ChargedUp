package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldCommand extends InstantCommand {
  private ArmSubsystem arm = ArmSubsystem.getInstance();
  public ArmHoldCommand() {
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.holdRotatePosition();
    arm.holdTelescopePosition();
  }
}
