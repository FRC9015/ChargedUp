package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PneumaticConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.LEDSubsystem.LEDPreset;

public class FootSubsystem extends SubsystemBase {

    private static FootSubsystem INSTANCE;
    private boolean forward;

    @SuppressWarnings("WeakerAccess")
    public static FootSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new FootSubsystem();

        return INSTANCE;
    }

    DoubleSolenoid foot;
    // PneumaticHub pHub;

    private FootSubsystem() {
        foot = PneumaticConstants.LIFT_FEET_CONSTANTS;
    }

    @Override
    public void periodic() {
        RobotState.setFeetDown(!(foot.get() == Value.kReverse));
    }

    public void footDown() {
        foot.set(DoubleSolenoid.Value.kForward);
    }

    public void footUp() {
        foot.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleFoot() {
        if (forward) {
            foot.set(DoubleSolenoid.Value.kReverse);
            forward = false;
            LEDSubsystem.getInstance().setPreset(LEDPreset.LOGOSLOW);

        } else {
            foot.set(DoubleSolenoid.Value.kForward);
            forward = true;
            LEDSubsystem.getInstance().setPreset(LEDPreset.RAINBOW);
        }
    }
}
