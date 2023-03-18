package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.utils.DoubleSolenoidConstants;

public class PneumaticHubSubsystem extends SubsystemBase {

    // Singleton Instance of this Class
    private static PneumaticHubSubsystem INSTANCE = new PneumaticHubSubsystem();
    
    // Getter Method for Singleton Instance
    public static PneumaticHubSubsystem getInstance() {
        return INSTANCE;
    }

    private PneumaticHub pHub;

    private PneumaticHubSubsystem() {
        pHub = new PneumaticHub(PneumaticConstants.P_HUB_CAN_ID);
    }

    public static Solenoid getSolenoid(int channel) {
        return INSTANCE.pHub.makeSolenoid(channel);
    }

    public static DoubleSolenoid getDoubleSolenoid(int forwardChannel, int reverseChannel) {
        return INSTANCE.pHub.makeDoubleSolenoid(forwardChannel, reverseChannel);
    }

    public static DoubleSolenoid getDoubleSolenoid(DoubleSolenoidConstants constants) {
        return INSTANCE.pHub.makeDoubleSolenoid(constants.getForwardChannel(), constants.getReverseChannel());
    }

    public void enableCompressor() {
        pHub.enableCompressorDigital();
    }

    public void disableCompressor() {
        pHub.disableCompressor();
    }

    public boolean isCompressorRunning() {
        return pHub.getCompressor();
    }

    public boolean isPressurized() {
        return pHub.getCompressor() && pHub.getPressureSwitch();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
