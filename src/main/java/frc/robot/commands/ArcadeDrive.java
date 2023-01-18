package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controllers.DriverController;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();
    private DriverController driver;

    public ArcadeDrive() {
        driver = RobotContainer.getInstance().getDriver();

        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void execute() {
        diffDriveSubsystem.arcadeDrive(driver.getArcadeFwd(), driver.getArcadeTurn());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            diffDriveSubsystem.stop();
        }
    }

}
