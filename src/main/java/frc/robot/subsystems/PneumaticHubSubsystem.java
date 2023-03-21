package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.utils.DoubleSolenoidConstants;
import frc.robot.utils.SolenoidBase;
import lombok.Synchronized;
import lombok.val;

public class PneumaticHubSubsystem extends SubsystemBase {

    // Singleton Instance of this Class
    private static PneumaticHubSubsystem INSTANCE;

    // Getter Method for Singleton Instance
    public static PneumaticHubSubsystem getInstance() {
        if (INSTANCE == null)
            INSTANCE = new PneumaticHubSubsystem();
        return INSTANCE;
    }

    private PneumaticHub pHub;
    private static ArrayList<SolenoidBase> solenoids = new ArrayList<SolenoidBase>();

    private PneumaticHubSubsystem() {
        pHub = new PneumaticHub(PneumaticConstants.P_HUB_CAN_ID);
    }

    public static Solenoid getSolenoid(int channel) {
        Solenoid solenoid = INSTANCE.pHub.makeSolenoid(channel);
        solenoids.add(new SolenoidBase(solenoid));
        return solenoid;
    }

    public static DoubleSolenoid getDoubleSolenoid(int forwardChannel, int reverseChannel) {
        DoubleSolenoid solenoid = INSTANCE.pHub.makeDoubleSolenoid(forwardChannel, reverseChannel);
        solenoids.add(new SolenoidBase(solenoid));
        return solenoid;
    }

    public static DoubleSolenoid getDoubleSolenoid(DoubleSolenoidConstants constants) {
        return getDoubleSolenoid(constants.getForwardChannel(), constants.getReverseChannel());
    }

    /**
     * Allows disabling of all solenoids. This is mainly used so they can be manually actuated for testing.
     */
    @Synchronized("solenoids")
    public void disableAllSolenoids() {
        for (SolenoidBase base : solenoids) {
            switch (base.getType()) {
                case Double:
                    val dSol = base.getDoubleSolenoid();
                    dSol.set(Value.kOff);
                case Single:
                    val sSol = base.getSingleSolenoid();
                    sSol.set(false);
            }
        }
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
