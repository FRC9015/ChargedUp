package frc.robot.commands.Arm.Lift;

public class ArmUpCommand extends ArmLiftCommand {
    public ArmUpCommand() {
        super(LiftDirection.Up);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
