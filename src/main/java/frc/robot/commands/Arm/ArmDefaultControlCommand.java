// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    private OperatorController controller;

    /** Creates a new armDefualtControlCommand. */
    public ArmDefaultControlCommand(OperatorController myController) {
        controller = myController;
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (Math.abs(controller.getTankLeft()) > 0.05) {
            armSubsystem.rotateArm(-controller.getTankLeft());
        } else {
            // armSubsystem.rotateArm(armSubsystem.getArmTorque()*0.005);
            armSubsystem.rotateArm(0);
        }

        if (Math.abs(controller.getTankRight()) > 0.05) {
            armSubsystem.telescopeArm(-0.45 * controller.getTankRight());
        } else {
            armSubsystem.telescopeArm(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
