package frc.robot.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class DriverController implements Sendable {

    private final DoubleSupplier tankLeft, tankRight, arcadeFwd, arcadeTurn;
    private final JoystickButton balanceButton, switchSpeed, homeWeightButton;
    private final GenericHID rawController;

    /** Create a driver controller with two standard Joysticks */
    public DriverController(Joystick joystick1, Joystick joystick2) {
        tankLeft = joystick1::getY;
        tankRight = joystick2::getY;
        arcadeFwd = joystick1::getY;
        arcadeTurn = joystick1::getX;

        homeWeightButton = new JoystickButton(joystick1, XboxController.Button.kB.value);

        balanceButton = null;
        switchSpeed = new JoystickButton(joystick1, Joystick.ButtonType.kTrigger.value);
        rawController = joystick1;
    }

    /** Create a driver controller with an XboxController */
    public DriverController(XboxController controller) {
        tankLeft = controller::getLeftY;
        tankRight = controller::getRightY;
        arcadeFwd = controller::getLeftY;
        arcadeTurn = controller::getRightX;

        balanceButton = new JoystickButton(controller, XboxController.Button.kA.value);
        homeWeightButton = new JoystickButton(controller, XboxController.Button.kB.value);
        switchSpeed = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);

        rawController = controller;
    }

    /** Create a driver controller with a PS4Controller */
    public DriverController(PS4Controller controller) {
        tankLeft = controller::getLeftY;
        tankRight = controller::getRightY;
        arcadeFwd = controller::getLeftY;
        arcadeTurn = controller::getRightX;

        homeWeightButton = new JoystickButton(controller, XboxController.Button.kB.value);

        balanceButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        switchSpeed = new JoystickButton(controller, PS4Controller.Button.kL3.value); // Left Joystick button

        rawController = controller;
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

    public JoystickButton getBalanceButton() {
        return balanceButton;
    }

    public JoystickButton getHomeWeightButton(){
        return homeWeightButton;
    }
    
    public JoystickButton getSwitchSpeed() {
        return switchSpeed;
    }

    public void setRumble(GenericHID.RumbleType rumble, double value) {
        rawController.setRumble(rumble, value);
    }
    
    // This allows us to send this DriverController to the Dashboard and read all of its values
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DriverController");
        builder.addDoubleProperty("arcadeFwd", arcadeFwd, null);
        builder.addDoubleProperty("arcadeTurn", arcadeTurn, null);
        builder.addDoubleProperty("tankLeft", tankLeft, null);
        builder.addDoubleProperty("tankLeft", tankRight, null);
        builder.addBooleanProperty("balanceButton", balanceButton::getAsBoolean, null);
    }

}
