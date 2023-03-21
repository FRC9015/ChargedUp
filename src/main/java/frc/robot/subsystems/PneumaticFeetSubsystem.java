// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class PneumaticFeetSubsystem extends SubsystemBase {

  private static PneumaticFeetSubsystem INSTANCE;

  public static PneumaticFeetSubsystem getInstance() {
    if (INSTANCE == null)
      INSTANCE = new PneumaticFeetSubsystem();
    return INSTANCE;
  }

  // -------------------

  private final DoubleSolenoid feet;
  public PneumaticFeetSubsystem() {
    feet = PneumaticHubSubsystem.getDoubleSolenoid(PneumaticConstants.LIFT_FEET_CONSTANTS);
  }

  @Override
  public void periodic() {}

  public void extendFeet() {
    feet.set(DoubleSolenoid.Value.kForward);
  }

  public void retractFeet() {
    feet.set(DoubleSolenoid.Value.kReverse);
  }

  public void disableFeet() {
    feet.set(DoubleSolenoid.Value.kOff);
  }

  public void toggleFeet() {
    switch(feet.get()) {
      case kForward:
        retractFeet();
      case kReverse:
        extendFeet();
      default:
        retractFeet();
    }
  }
}
