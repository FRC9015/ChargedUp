package frc.robot.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
public class DriverController implements Sendable {

    private final DoubleSupplier tankLeft, tankRight, arcadeFwd, arcadeTurn;

    /** Create a driver controller with a standard Joystick */
    public DriverController(Joystick joystick1, Joystick joystick2) {
        tankLeft = joystick1::getY;
        tankRight = joystick2::getY;
        arcadeFwd = joystick1::getY;
        arcadeTurn = joystick1::getX;
    }

    /** Create a driver controller with an XboxController */
    public DriverController(XboxController controller) {
        tankLeft = controller::getLeftY;
        tankRight = controller::getRightY;
        arcadeFwd = controller::getLeftY;
        arcadeTurn = controller::getRightX;
    }

    /** Create a driver controller with a PS4Controller */
    public DriverController(PS4Controller controller) {
        tankLeft = controller::getLeftY;
        tankRight = controller::getRightY;
        arcadeFwd = controller::getLeftY;
        arcadeTurn = controller::getRightX;

    }

    public double getTankLeft() {
        return tankLeft.getAsDouble();
    }

    public double getTankRight() {
        return tankRight.getAsDouble();
    }

    public double getArcadeFwd() {
        return arcadeFwd.getAsDouble();
    }

    public double getArcadeTurn() {
        return arcadeTurn.getAsDouble();
    }
    
    // This allows us to send this DriverController to the Dashboard and read all of its values
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DriverController");
        builder.addDoubleProperty("arcadeFwd", arcadeFwd, null);
        builder.addDoubleProperty("arcadeTurn", arcadeTurn, null);
        builder.addDoubleProperty("tankLeft", tankLeft, null);
        builder.addDoubleProperty("tankLeft", tankRight, null);
    }

}
