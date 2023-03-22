package frc.robot.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;

public class OperatorController implements Sendable {

    private final GenericHID rawController;

    private DoubleSupplier joyLeftX, joyLeftY, joyRightX, joyRightY, leftTriggerValue, rightTriggerValue;
    @Getter
    private JoystickButton A, B, X, Y, start, back, leftBumper, rightBumper, leftTrigger, rightTrigger;
    @Getter
    private POVButton povUp, povDown, povLeft, povRight;

    public OperatorController(XboxController controller) {
        joyLeftY = controller::getLeftY;
        joyLeftX = controller::getLeftX;
        joyRightY = controller::getRightY;
        joyRightX = controller::getRightX;

        leftTriggerValue = controller::getLeftTriggerAxis;
        rightTriggerValue = controller::getRightTriggerAxis;

        povUp = new POVButton(controller, 0);
        povRight = new POVButton(controller, 90);
        povDown = new POVButton(controller, 180);
        povLeft = new POVButton(controller, 270);

        A = new JoystickButton(controller, XboxController.Button.kA.value);
        B = new JoystickButton(controller, XboxController.Button.kB.value);
        X = new JoystickButton(controller, XboxController.Button.kX.value);
        Y = new JoystickButton(controller, XboxController.Button.kY.value);

        leftBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

        start = new JoystickButton(controller, XboxController.Button.kStart.value);
        back = new JoystickButton(controller, XboxController.Button.kBack.value);
        
        leftTrigger = new JoystickButton(controller, XboxController.Axis.kLeftTrigger.value);
        rightTrigger = new JoystickButton(controller, XboxController.Axis.kRightTrigger.value);


        rawController = controller;
    }

    // Quick mockup of Joystick controls for Bella
    public OperatorController(Joystick controller) {
        joyLeftY = controller::getY;
        joyLeftX = controller::getX;
        joyRightY = controller::getY;
        joyRightX = controller::getX;

        povUp = new POVButton(controller, 0);
        povRight = new POVButton(controller, 90);
        povDown = new POVButton(controller, 180);
        povLeft = new POVButton(controller, 270);

        leftTriggerValue = () -> controller.getTriggerPressed() ? 1.0 : 0.0;

        rawController = controller;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Operator");
        
    }

    public double getLeftX() {
        return joyLeftX.getAsDouble();
    }

    public double getLeftY() {
        return joyLeftY.getAsDouble();
    }

    public double getRightX() {
        return joyRightX.getAsDouble();
    }

    public double getRightY() {
        return joyRightY.getAsDouble();
    }

    public Trigger getLTriggerAsButton(){
        return new Trigger(()-> leftTriggerValue.getAsDouble() > 0.3);
    }

    public Trigger getRTriggerAsButton(){
        return new Trigger(()-> rightTriggerValue.getAsDouble() > 0.3);
    }

    public void setRumble(GenericHID.RumbleType rumble, double value) {
        rawController.setRumble(rumble, value);
    }
}
