package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
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

    private PneumaticsControlModule pHub;

    private PneumaticHubSubsystem() {
        pHub = new PneumaticsControlModule(PneumaticConstants.P_HUB_CAN_ID);
    }

    public Solenoid getDoubleSolenoid(int channel) {
        return pHub.makeSolenoid(channel);
    }

    public DoubleSolenoid getDoubleSolenoid(int forwardChannel, int reverseChannel) {
        return pHub.makeDoubleSolenoid(forwardChannel, reverseChannel);
    }

    public DoubleSolenoid getDoubleSolenoid(DoubleSolenoidConstants constants) {
        return pHub.makeDoubleSolenoid(constants.getForwardChannel(), constants.getReverseChannel());
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
