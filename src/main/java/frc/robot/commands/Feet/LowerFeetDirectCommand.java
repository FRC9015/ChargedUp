package frc.robot.commands.Feet;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticFeetSubsystem;

public class LowerFeetDirectCommand extends CommandBase {

  PneumaticFeetSubsystem pFeet = PneumaticFeetSubsystem.getInstance();

  private boolean fin = false;
  
  public LowerFeetDirectCommand() {
    addRequirements(pFeet);
  }

  @Override
  public void initialize() {
    pFeet.retractFeet();
    fin = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
