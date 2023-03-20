package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RobotState;

public class SlowedWhileActiveCommand extends StartEndCommand {
  public SlowedWhileActiveCommand() {
    super(RobotState::setSlow, RobotState::setFast);
  }
}
