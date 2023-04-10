package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.controllers.DriverController;
import frc.robot.subsystems.DiffDriveSubsystem;

public class TankDrive extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();

    private DriverController driver;

    public TankDrive(DriverController myDriver) {
        driver = myDriver;

        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void execute() {
        diffDriveSubsystem.tankDrive(driver.getTankLeft(), driver.getTankRight());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            diffDriveSubsystem.stop();
        }
    }
}
