package frc.robot.utils;

import lombok.Getter;

public class DoubleSolenoidConstants {
    @Getter private final int forwardChannel, reverseChannel;

    public DoubleSolenoidConstants(int forwardChannel, int reverseChannel) {
        this.forwardChannel = forwardChannel;
        this.reverseChannel = reverseChannel;
    }
}
