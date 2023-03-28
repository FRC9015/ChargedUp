package frc.robot.commands.Arm.Lift;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.ArmMoveCommand;
import lombok.NonNull;

public class ArmLiftCommand extends ArmMoveCommand {
    public static enum LiftDirection {
        Up,
        Down,
    }

    public ArmLiftCommand(LiftDirection direction) {
        super(getDefaultSpeed(direction), 0);
    }
 
    public ArmLiftCommand(double liftSpeed) {
        super(liftSpeed, 0);
    }

    private static double getDefaultSpeed(@NonNull LiftDirection direction) {
        switch (direction) {
            case Up:
                return ArmConstants.DEFAULT_LIFT_SPEED;
            case Down:
                return -ArmConstants.DEFAULT_LIFT_SPEED;
            default:
                return ArmConstants.DEFAULT_LIFT_SPEED;
        }
    }
}
