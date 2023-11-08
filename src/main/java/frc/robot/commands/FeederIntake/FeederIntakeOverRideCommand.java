package frc.robot.commands.FeederIntake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.FeederIntakeSubsystem;

public class FeederIntakeOverRideCommand implements Command {
    private final FeederIntakeSubsystem feederIntakeSubsystem = FeederIntakeSubsystem.getInstance();
    private final Set<Subsystem> subsystems;

    public FeederIntakeOverRideCommand() {
        this.subsystems = Set.of(this.feederIntakeSubsystem);
    }

    public void initialize() {
        feederIntakeSubsystem.feederIntakeOverRide();
    }

    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}