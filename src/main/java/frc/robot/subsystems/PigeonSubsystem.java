package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;
import frc.robot.Constants.SensorConstants;

public class PigeonSubsystem extends SubsystemBase {
    

    private final static PigeonSubsystem INSTANCE = new PigeonSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static PigeonSubsystem getInstance() {
        return INSTANCE;
    }

    private final WPI_Pigeon2 pigeonSensor;

    private PigeonSubsystem() {
        pigeonSensor = new WPI_Pigeon2(SensorConstants.PIGEON_CAN_ID);
    }

    public double getXTilt() {
        System.out.println(pigeonSensor.getPitch());
        return pigeonSensor.getPitch();
    }

    public Rotation2d getRotation2d() {
        return pigeonSensor.getRotation2d();
    }

    public void resetAngles() {
        pigeonSensor.zeroGyroBiasNow();
    }

    public void resetHeading() {
        pigeonSensor.reset();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().balance.setAngle(getXTilt());
    }

}
