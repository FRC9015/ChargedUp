package frc.robot.commands;

import frc.robot.Dashboard;
import frc.robot.Helpers;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleBalanceCommand extends CommandBase {





  private PigeonSubsystem pigeon;
  private DiffDriveSubsystem drive;
  private FootSubsystem foot;

  private int filtersize=10;

  private MedianFilter angleFilter;

  private boolean finished = false;
    

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

    //drives in the direction to balance and when its balanced it stops
    if (angle>=0.1){
      drive.arcadeDriveRaw(0.1, 0, false);
    }else if (angle<-0.1){
      drive.arcadeDriveRaw(-0.1, 0, false);
    }else{
      drive.arcadeDriveRaw(0,0,false);
      foot.footDown();
      finished=true;
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
