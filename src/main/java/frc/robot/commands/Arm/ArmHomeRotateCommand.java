package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHomeRotateCommand extends InstantCommand {
  private final ArmSubsystem arm = ArmSubsystem.getInstance();

  public ArmHomeRotateCommand() {
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.homeRotate();
  }
}
