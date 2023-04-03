package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class ArcadeDrivecopy extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();
    private DriverController driver;

    public ArcadeDrivecopy(DriverController myDriver) {
        driver = myDriver;
        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void execute() {
        diffDriveSubsystem.arcadeDriveRawer(driver.getArcadeFwd(), driver.getArcadeTurn(),true);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            diffDriveSubsystem.stop();
        }
    }

}
