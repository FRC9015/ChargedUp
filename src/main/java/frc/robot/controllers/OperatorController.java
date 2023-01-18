package frc.robot.controllers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorController implements Sendable {

    /** Create an operator controller with an XboxController */
    public OperatorController(XboxController controller) {

    }

    /** Create an operator controller with a PS4Controller */
    public OperatorController(PS4Controller controller) {

    }

    // This allows us to send this OperatorController to the Dashboard and read all of its values
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("OperatorController");
    }

}
