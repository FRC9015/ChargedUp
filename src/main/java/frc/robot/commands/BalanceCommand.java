package frc.robot.commands;

import frc.robot.Helpers;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {

    PigeonSubsystem pigeon;
    DiffDriveSubsystem drive;

    double kP = 0, kI = 0, kD = 0;
    PIDController balancePID = new PIDController(kP, kI, kD);
    

  public BalanceCommand(PigeonSubsystem newPigeon, DiffDriveSubsystem newDrive) {
    this.pigeon = newPigeon;
    this.drive = newDrive;
    
    addRequirements(pigeon, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.tankDrive(0, 0);
    drive.setBrakeMode(IdleMode.kBrake);

    balancePID.setSetpoint(0.0);
    balancePID.setTolerance(2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = pigeon.getXTilt();

    double moveSpeed = balancePID.calculate(angle);

    // Limit max motor speed to 0.2
    drive.arcadeDrive(Helpers.limitDecimal(moveSpeed, 0.2), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.brakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
