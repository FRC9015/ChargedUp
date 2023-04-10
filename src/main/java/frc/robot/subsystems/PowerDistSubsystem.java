// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistSubsystem extends SubsystemBase {
    private static PowerDistSubsystem INSTANCE;

    public static PowerDistSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new PowerDistSubsystem();

        return INSTANCE;
    }

    private PowerDistribution pdh;
    private int PDH_CAN_ID = 2;

    private PowerDistSubsystem() {
        pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);
    }

    public void setSwitchable(boolean isOn) {
        pdh.setSwitchableChannel(isOn);
    }

    public boolean getSwitchable() {
        return pdh.getSwitchableChannel();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
