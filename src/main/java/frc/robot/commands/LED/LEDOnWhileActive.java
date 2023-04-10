// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.PowerDistSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDOnWhileActive extends StartEndCommand {
    public LEDOnWhileActive() {
        super(
                () -> PowerDistSubsystem.getInstance().setSwitchable(true),
                () -> PowerDistSubsystem.getInstance().setSwitchable(false),
                PowerDistSubsystem.getInstance());
    }
}
