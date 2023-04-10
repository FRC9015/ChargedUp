// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakePneumaticSubsystem;

public class RunIntakeMotorCommand extends CommandBase {
    public IntakePneumaticSubsystem intake = IntakePneumaticSubsystem.getInstance();
    double speed;

    public RunIntakeMotorCommand(double atSpeed) {
        addRequirements(intake);
        speed = atSpeed;
    }

    @Override
    public void initialize() {
        intake.setIntakeMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeMotorSpeed(0);
    }
}
