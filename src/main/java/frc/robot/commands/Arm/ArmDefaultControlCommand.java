package frc.robot.commands.Arm;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.controllers.OperatorController;

public class ArmDefaultControlCommand extends ArmMoveCommand {
    private OperatorController op;
    public ArmDefaultControlCommand() {
        super(0, 0);
        op = RobotContainer.getInstance().getOperator();
    }

    @Override
    public void execute() {
        setArmLiftSpeed(op.getLeftY()*ArmConstants.LIFT_INPUT_SCALAR);
        setArmTelescopeSpeed(op.getRightY()*ArmConstants.TELESCOPE_INPUT_SCALAR);
        super.execute();
    }
}
