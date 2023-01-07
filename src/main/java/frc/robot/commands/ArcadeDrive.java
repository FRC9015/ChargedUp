package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class ArcadeDrive extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem = DiffDriveSubsystem.getInstance();

    public ArcadeDrive() {
        addRequirements(this.diffDriveSubsystem);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        diffDriveSubsystem.arcadeDrive(RobotContainer.getInstance().getDriverJoystick().getY(), RobotContainer.getInstance().getDriverJoystick().getX());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        diffDriveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
