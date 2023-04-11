package frc.robot.utils;

import lombok.Getter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidBase {
    public enum SolenoidType {
        Double,
        Single
    }

    @Getter
    private DoubleSolenoid doubleSolenoid;

    @Getter
    private Solenoid singleSolenoid;

    @Getter
    private final SolenoidType type;

    public SolenoidBase(DoubleSolenoid solenoid) {
        doubleSolenoid = solenoid;
        type = SolenoidType.Double;
    }

    public SolenoidBase(Solenoid solenoid) {
        singleSolenoid = solenoid;
        type = SolenoidType.Single;
    }
}
