package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

import java.util.Set;

public class dumbDriveCommand implements Command {
    private final DiffDriveSubsystem diffDriveSubsystem;
    private final Set<Subsystem> subsystems;



    private State initialState;
    private double desiredDistance;
    private TrapezoidProfile profile;
    private double time =0;

    public dumbDriveCommand(DiffDriveSubsystem mydiffDriveSubsystem, double distance) {
        this.diffDriveSubsystem = mydiffDriveSubsystem;
        this.desiredDistance = distance;
        this.subsystems = Set.of(this.diffDriveSubsystem);
    }

    @Override
    public void initialize() {
        initialState = new State(0, 0.0);


        State finalState = new State(desiredDistance, 0.0);

        //might be wrong acceleration
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(2, 1),
            finalState, 
            initialState);

        System.out.println(profile.totalTime());
        System.out.println(initialState.position-finalState.position);

    }

    @Override
    public void execute() {
        State predictedState = profile.calculate(time/50);

        double targetVelocity = predictedState.velocity;



        


        diffDriveSubsystem.setSpeeds(new DifferentialDriveWheelSpeeds(targetVelocity, targetVelocity));
        


        time++;
    }

    @Override
    public boolean isFinished() {
        return (profile.isFinished(time/50));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
