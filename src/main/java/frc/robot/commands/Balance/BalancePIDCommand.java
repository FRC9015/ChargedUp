// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Dashboard;
import frc.robot.Helpers;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.PigeonSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalancePIDCommand extends PIDCommand {

    private static double kP = 0.5, kI = 0, kD = 0;

    private static PigeonSubsystem pigeon = PigeonSubsystem.getInstance();
    private static DiffDriveSubsystem drive = DiffDriveSubsystem.getInstance();
    private static MedianFilter angleFilter =
            new MedianFilter(
                    25); // Robot refreshes at ~50Hz, so average over the last half second of
    // measurements

    public BalancePIDCommand() {
        super(
                new PIDController(kP, kI, kD),
                () -> {
                    return angleFilter.calculate(pigeon.getPitch());
                },
                0.0, // The constant setpoint
                output -> {
                    drive.arcadeDriveRaw(Helpers.limitDecimal(-output, 0.25), 0, false);
                },
                pigeon,
                drive // Subsystem requirements
                );

        getController().setTolerance(4.0);
    }

    @Override
    public void initialize() {
        super.initialize();

        drive.tankDriveRaw(0, 0, false);
        drive.setBrakeMode(IdleMode.kBrake);

        angleFilter.reset();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (!interrupted) {
            // If the command finished without being interrupted, then set the s
            Dashboard.getInstance().balance.setAutoBalanced(true);
        } else if (interrupted) {
            Dashboard.getInstance().balance.setAutoBalanced(false);
        }
        drive.brakeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
