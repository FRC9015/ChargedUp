package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHomeTelescopeCommand extends InstantCommand {
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  boolean isRetracted;

  /**
   * @param retracted If true, set the soft-limit for the telescope fully
   *                  retracted. If false, set the soft limit for the telescope
   *                  fully extended.
   */
  public ArmHomeTelescopeCommand(boolean retracted) {
    addRequirements(arm);
    isRetracted = retracted;
  }

  @Override
  public void initialize() {
    arm.homeTelescope(isRetracted);
  }
}
