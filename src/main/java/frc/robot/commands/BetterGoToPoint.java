package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

public class BetterGoToPoint extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final Set<Subsystem> subsystems;

    public BetterGoToPoint(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.subsystems = Set.of(this.limelightSubsystem);
    }

    @Override
    public void initialize() {
      double[] botpose = limelightSubsystem.getBotpose();
      double x = (botpose[0]); // Horizontal displacement
      double y = (botpose[1]); // Vertical displacement
      double dx = (3 - x);
      double dy = (2 - y);
      double hyp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
      
    }

    @Override
    public void execute() {
      double[] botpose = limelightSubsystem.getBotpose();
      System.out.print(botpose[0]);
      System.out.print(botpose[1]);
      System.out.print(botpose[2]);
      System.out.print(botpose[3]);
      System.out.print(botpose[4]);
      System.out.print(botpose[5]);  
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
