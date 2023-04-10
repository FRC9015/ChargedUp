package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.controllers.DriverController;
import frc.robot.subsystems.DiffDriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();
    private DriverController driver;

    public ArcadeDrive(DriverController myDriver) {
        driver = myDriver;
        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void execute() {
        diffDriveSubsystem.arcadeDrive(driver.getArcadeFwd(), driver.getArcadeTurn());

        // DifferentialDriveWheelSpeeds speeds = diffDriveSubsystem.getWheelSpeeds();

        // Helpers.logBox("Fwd: " + driver.getArcadeFwd(), "Turn: " + driver.getArcadeTurn(), "Left:
        // " + speeds.leftMetersPerSecond, "Right: " + speeds.rightMetersPerSecond );
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            diffDriveSubsystem.stop();
        }
    }
}
