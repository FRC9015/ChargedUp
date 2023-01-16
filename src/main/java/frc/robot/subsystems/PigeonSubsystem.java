package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;

public class PigeonSubsystem extends SubsystemBase {
    

    private final static PigeonSubsystem INSTANCE = new PigeonSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static PigeonSubsystem getInstance() {
        return INSTANCE;
    }

    private final Pigeon2 pigeonSensor;

    private PigeonSubsystem() {
        pigeonSensor = new Pigeon2(SensorConstants.PIGEON_CAN_ID);
    }

    public double getXTilt() {
        System.out.println(pigeonSensor.getPitch());
        return pigeonSensor.getPitch();
    }

    public void resetAngles() {
        pigeonSensor.zeroGyroBiasNow();
    }

}
