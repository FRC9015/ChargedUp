package frc.robot.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements Sendable {

    private final GenericHID rawController;
    private final DoubleSupplier tankLeft, tankRight, arcadeFwd, arcadeTurn, lTrigger, rTrigger;
    private final JoystickButton a, b, x, y, start, back, ls, rs, lb, rb;
    private final POVButton up, down, left, right;

    public OperatorController(XboxController controller) {
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

    public JoystickButton getA() {
        return a;
    }

    public JoystickButton getB() {
        return b;
    }

    public JoystickButton getX() {
        return x;
    }

    public JoystickButton getY() {
        return y;
    }

    public JoystickButton getStart() {
        return start;
    }

    public JoystickButton getBack() {
        return back;
    }

    public JoystickButton getLS() {
        return ls;
    }

    public JoystickButton getRS() {
        return rs;
    }

    public JoystickButton getRB() {
        return rb;
    }

    public JoystickButton getLB() {
        return lb;
    }

    public Trigger getLTrigAsButton() {
        return new Trigger(() -> lTrigger.getAsDouble() > 0.3);
    }

    public Trigger getRTrigAsButton() {
        return new Trigger(() -> rTrigger.getAsDouble() > 0.3);
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

    @Override
    public void initSendable(SendableBuilder builder) {}
}
