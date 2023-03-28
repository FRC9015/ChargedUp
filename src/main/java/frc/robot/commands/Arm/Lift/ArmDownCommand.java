package frc.robot.commands.Arm.Lift;

public class ArmDownCommand extends ArmLiftCommand {
    public ArmDownCommand() {
        super(LiftDirection.Down);
    }    

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
