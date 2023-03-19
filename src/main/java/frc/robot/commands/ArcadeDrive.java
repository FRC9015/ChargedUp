package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();
    private DriverController driver;

    public ArcadeDrive(DriverController myDriver) {
        driver = myDriver;
        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void execute() {
        diffDriveSubsystem.arcadeDrive(driver.getArcadeFwd()*0.8, driver.getArcadeTurn()*0.8);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            diffDriveSubsystem.stop();
        }
    }

}
