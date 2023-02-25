package frc.robot.commands;

import frc.robot.Dashboard;
import frc.robot.Helpers;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {

  private PigeonSubsystem pigeon;
  private DiffDriveSubsystem drive;

  private double kP = 0.007, kI = 0, kD = 0.001;
  private PIDController balancePID = new PIDController(kP, kI, kD);

  private MedianFilter angleFilter;

  private boolean finished = false;
    

  public BalanceCommand(PigeonSubsystem newPigeon, DiffDriveSubsystem newDrive) {
    pigeon = newPigeon;
    drive = newDrive;

    angleFilter = new MedianFilter(25); // Robot refreshes at ~50Hz, so average over the last half second of measurements
    
    addRequirements(pigeon, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.tankDriveRaw(0, 0, false);
    drive.setBrakeMode(IdleMode.kBrake);

    balancePID.setSetpoint(0.0); // 0 degrees is obviously balanced
    balancePID.setTolerance(2.0); // Allow for 1 degrees of error in either direction
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // angleFilter calculates a moving average of the angle to correct for any spikes
    double angle = angleFilter.calculate(pigeon.getPitch());

    double moveSpeed = balancePID.calculate(angle);
    // Limit max motor speed to 0.2
    drive.arcadeDriveRaw(-Helpers.limitDecimal(moveSpeed, 0.2), 0, false);

    if (Math.abs(moveSpeed) <= 0.01) {
      // If the PID is calculating below a certain threshold, we can safely assume that we're balanced
      //finished = true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
    return finished;
  }
}
