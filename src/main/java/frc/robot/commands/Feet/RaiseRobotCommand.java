package frc.robot.commands.Feet;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitForPressureCommand;

/**
 * Wait for the system to pressurize (Digital sensor reading pressurized and compressor off) then extend feet, raising the robot
 */
public class RaiseRobotCommand extends SequentialCommandGroup {
  /** Creates a new RaiseRobotCommand. */
  public RaiseRobotCommand() {
    addCommands(new WaitForPressureCommand(), new RaiseFeetDirectCommand());
  }
}
