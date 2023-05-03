package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

import java.util.Set;

public class turnCommand implements Command {
    private final DiffDriveSubsystem diffDriveSubsystem;
    private final PigeonSubsystem pigeonSubsystem;
    private final Set<Subsystem> subsystems;


    private State initialState;
    private double desiredAngle;
    private TrapezoidProfile profile;
    private double time =0;
    private PIDController pid = new PIDController(0.0002, 0, 0);

    public turnCommand(DiffDriveSubsystem mydiffDriveSubsystem, PigeonSubsystem mypigeonSubsystem,double rotationAngle) {
        this.pigeonSubsystem = mypigeonSubsystem;
        this.diffDriveSubsystem = mydiffDriveSubsystem;
        desiredAngle = rotationAngle;
        this.subsystems = Set.of(this.pigeonSubsystem,this.diffDriveSubsystem);
    }

    @Override
    public void initialize() {
        initialState = new State(pigeonSubsystem.getAngle(), 0.0);
        time =0;

        //the weird number i multiply by is to convert degrees to meters
        State finalState = new State(pigeonSubsystem.getAngle()+desiredAngle*0.0035429138888889, 0.0);


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
        double currentAngle = pigeonSubsystem.getAngle();
        State predictedState = profile.calculate(time/50);

        double targetVelocity = predictedState.velocity;
        double targetPosition = predictedState.position;

        double angle = initialState.position+(targetPosition-initialState.position)/0.0035429138888889;

        double correctoin = pid.calculate(currentAngle, angle);

        


        diffDriveSubsystem.setSpeeds(new DifferentialDriveWheelSpeeds(targetVelocity+correctoin, -(targetVelocity+correctoin)));
        //diffDriveSubsystem.setSpeeds(new DifferentialDriveWheelSpeeds(correctoin, -(correctoin)));
   


        time++;
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(time/50);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
