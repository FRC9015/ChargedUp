package frc.robot.commands;

import frc.robot.Dashboard;
import frc.robot.Helpers;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleBalanceCommand extends CommandBase {


  boolean pidActive;


  private PigeonSubsystem pigeon;
  private DiffDriveSubsystem drive;
  private FootSubsystem foot;

  private int filtersize=30;

  private MedianFilter angleFilter;

  private boolean finished = false;
    
  private double kP = 0.005, kI = 0, kD = 0.0008;


  private PIDController balancePID = new PIDController(kP, kI, kD);

  public SimpleBalanceCommand(PigeonSubsystem newPigeon, DiffDriveSubsystem newDrive,FootSubsystem newfootSubsystem) {


    pigeon = newPigeon;
    drive = newDrive;
    foot = newfootSubsystem;

    



    //angleFilter = new MedianFilter((int)SmartDashboard.getNumber("bal filter", 20)); // Robot refreshes at ~50Hz, so average over the last half second of measurements
    angleFilter = new MedianFilter(filtersize); // Robot refreshes at ~50Hz, so average over the last half second of measurements


    addRequirements(pigeon, drive,foot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.tankDriveRaw(0, 0, false);
    drive.setBrakeMode(IdleMode.kBrake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //finds and filters the angle
    double angle = angleFilter.calculate(pigeon.getPitch());
    System.out.println(angle);

    if(pidActive){

      double moveSpeed = balancePID.calculate(angle);

      drive.arcadeDriveRaw(Helpers.limitDecimal(-moveSpeed, 0.2), 0, false);

    }else{
    //drives in the direction to balance and when its balanced it stops
    if (pigeon.getPitch()>=1){
      drive.arcadeDriveRaw(0.1, 0, false);
    }else if (pigeon.getPitch()<-1){
      drive.arcadeDriveRaw(-0.1, 0, false);
    }else{
      pidActive = true;
    }


  }}

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
