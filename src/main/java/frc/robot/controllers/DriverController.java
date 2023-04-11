package frc.robot.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import lombok.Getter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverController implements Sendable {

    private final GenericHID rawController;

    private final DoubleSupplier tankLeft, tankRight, arcadeFwd, arcadeTurn;
    private final DoubleSupplier lTrigger, rTrigger;
    @Getter
    private final JoystickButton a, b, x, y, start, back, ls, rs, lb, rb, lTrig, rTrig;
    private final POVButton up, down, left, right;

    public DriverController(XboxController controller) {
        tankLeft = controller::getLeftY;
        tankRight = controller::getRightY;
        arcadeFwd = controller::getLeftY;
        arcadeTurn = controller::getRightX;

        lTrigger = controller::getLeftTriggerAxis;
        rTrigger = controller::getRightTriggerAxis;

        up = new POVButton(controller, 0);
        down = new POVButton(controller, 180);
        left = new POVButton(controller, 270);
        right = new POVButton(controller, 90);

        a = new JoystickButton(controller, XboxController.Button.kA.value);
        b = new JoystickButton(controller, XboxController.Button.kB.value);
        x = new JoystickButton(controller, XboxController.Button.kX.value);
        y = new JoystickButton(controller, XboxController.Button.kY.value);

        lb = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        rb = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

        ls = new JoystickButton(controller, XboxController.Button.kLeftStick.value);
        rs = new JoystickButton(controller, XboxController.Button.kRightStick.value);

        start = new JoystickButton(controller, XboxController.Button.kStart.value);
        back = new JoystickButton(controller, XboxController.Button.kBack.value);

        lTrig = new JoystickButton(controller, XboxController.Axis.kLeftTrigger.value);
        rTrig = new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);

        rawController = controller;
    }

    public Trigger getUpDpad() {
        return up;
    }

    public Trigger getDownDpad() {
        return down;
    }

    public Trigger getLeftDpad() {
        return left;
    }

    public Trigger getRightDpad() {
        return right;
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

    public JoystickButton getLTrigAsButton() {
        return lTrig;
    }

    public Trigger getRTrigAsButton() {
        return new Trigger(new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                return rTrigger.getAsDouble() > 0.3;
            }
        });
    }

    public double getLTrigger() {
        return lTrigger.getAsDouble();
    }

    public double getRTrigger() {
        return rTrigger.getAsDouble();
    }

    public void setRumble(GenericHID.RumbleType rumble, double value) {
        rawController.setRumble(rumble, value);
    }

    // This allows us to send this DriverController to the Dashboard and read all of
    // its values
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DriverController");
        builder.addDoubleProperty("arcadeFwd", arcadeFwd, null);
        builder.addDoubleProperty("arcadeTurn", arcadeTurn, null);
        builder.addDoubleProperty("tankLeft", tankLeft, null);
        builder.addDoubleProperty("tankLeft", tankRight, null);
        builder.addBooleanProperty("ltrigger", lTrig, null);
        // builder.addBooleanProperty("balanceButton", balanceButton::getAsBoolean, null);
    }
}
