package frc.robot.commands.Feet;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticFeetSubsystem;

public class RaiseFeetDirectCommand extends CommandBase {

  PneumaticFeetSubsystem pFeet = PneumaticFeetSubsystem.getInstance();

  private boolean fin = false;
  
  public RaiseFeetDirectCommand() {
    addRequirements(pFeet);
  }

  @Override
  public void initialize() {
    pFeet.extendFeet();
    fin = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
