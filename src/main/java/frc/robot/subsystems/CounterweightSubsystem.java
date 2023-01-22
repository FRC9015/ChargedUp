// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.CounterweightConstants;

public class CounterweightSubsystem extends SubsystemBase {
  private static CounterweightSubsystem INSTANCE = new CounterweightSubsystem();

  public static CounterweightSubsystem getInstance() {
    return INSTANCE;
  }

  TalonSRX drive;
  DigitalInput endstop;
  Encoder encoder;

  /** Creates a new CounterweightSubsystem. */
  private CounterweightSubsystem() {
    drive = new TalonSRX(CounterweightConstants.DRIVE_MOTOR_CAN_ID);
    drive.setNeutralMode(NeutralMode.Brake);

    endstop = new DigitalInput(CounterweightConstants.ENDSTOP_SWITCH_PORT);

    encoder = new Encoder(CounterweightConstants.ENCODER_DIO_PINS[0], CounterweightConstants.ENCODER_DIO_PINS[1]);
  }

  public void zeroEncoder() {
    encoder.reset();
  }

  @Override
  public void periodic() {
    Dashboard.getInstance().counterweight.setEndstop(endstop.get());
  }
}
