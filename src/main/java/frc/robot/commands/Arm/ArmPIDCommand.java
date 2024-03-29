// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPIDCommand extends CommandBase {

    private PIDController rotPidController, telePidController;
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    ;
    private final Set<Subsystem> subsystems;
    private boolean waitToExtend;

    private OperatorController controller;

    /** Creates a new armpid. */
    public ArmPIDCommand(
            double rotsetpoint,
            double telesetpoint,
            boolean waitToExtend,
            double error,
            OperatorController opController) {
        rotPidController = new PIDController(3, 0.5, 0.3);

        rotPidController.setSetpoint(rotsetpoint);
        rotPidController.setTolerance(error);

        telePidController = new PIDController(0.9, 0.1, 0.2);
        telePidController.setSetpoint(telesetpoint);
        telePidController.setTolerance(error);

        this.waitToExtend = waitToExtend;

        controller = opController;

        this.subsystems = Set.of(this.armSubsystem);
    }

    public void execute() {

        if (Math.abs(controller.getTankLeft()) > 0.05) {
            rotPidController.setSetpoint(
                    rotPidController.getSetpoint() - controller.getTankLeft() * 0.01);
        }

        if (Math.abs(controller.getTankRight()) > 0.05) {
            telePidController.setSetpoint(
                    telePidController.getSetpoint() - controller.getTankRight() * 0.01);
            ;
        }

        armSubsystem.rotateArm(
                Math.max(
                        Math.min(0.6, rotPidController.calculate(armSubsystem.getRotEncoderPos())),
                        -0.6));
        if (waitToExtend) {
            if (armSubsystem.getRotEncoderPos() > 0.3) {
                armSubsystem.telescopeArm(
                        telePidController.calculate(armSubsystem.getTeleEncoderPos()));
            }
        } else {
            armSubsystem.telescopeArm(
                    telePidController.calculate(armSubsystem.getTeleEncoderPos()));
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (telePidController.atSetpoint() && rotPidController.atSetpoint()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.rotateArm(0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
