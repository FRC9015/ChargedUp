// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Telescope;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.ArmMoveCommand;
import lombok.NonNull;

/** Add your docs here. */
public class ArmTelescopeCommand extends ArmMoveCommand {
    public static enum TelescopeDirection {
        In,
        Out
    }

    public ArmTelescopeCommand(TelescopeDirection direction) {
        super(0, getDefaultSpeed(direction));
    }

    public ArmTelescopeCommand(double telescopeSpeed) {
        super(0, telescopeSpeed);
    }

    private static double getDefaultSpeed(@NonNull TelescopeDirection direction) {
        switch (direction) {
            case In:
                return ArmConstants.DEFAULT_TELESCOPE_SPEED;
            case Out:
                return -ArmConstants.DEFAULT_TELESCOPE_SPEED;
            default:
                return ArmConstants.DEFAULT_TELESCOPE_SPEED;
        }
    }
}
