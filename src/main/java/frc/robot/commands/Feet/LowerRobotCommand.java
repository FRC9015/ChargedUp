package frc.robot.commands.Feet;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Retract the feet, lowering the Robot
 */
public class LowerRobotCommand extends SequentialCommandGroup {
  public LowerRobotCommand() {
    addCommands(new LowerFeetDirectCommand());
  }
}
