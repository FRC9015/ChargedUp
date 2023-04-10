package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SensorConstants;
import frc.robot.Dashboard;

public class PigeonSubsystem extends SubsystemBase {
    private static PigeonSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static PigeonSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new PigeonSubsystem();
        return INSTANCE;
    }

    private final WPI_Pigeon2 pigeonSensor;

    private PigeonSubsystem() {
        pigeonSensor = new WPI_Pigeon2(SensorConstants.PIGEON_CAN_ID);
        addChild("pigeon", pigeonSensor);
    }

    /**
     * Get the pitch (forward/backward angle) of the robot
     *
     * @return double angle in degrees
     */
    public double getPitch() {
        return pigeonSensor.getPitch();
    }

    public Rotation2d getRotation2d() {
        return pigeonSensor.getRotation2d();
    }

    public double getAngle() {
        return pigeonSensor.getAngle();
    }

    public void resetAngles() {
        pigeonSensor.zeroGyroBiasNow();
    }

    public void resetHeading() {
        pigeonSensor.reset();
    }

    @Override
    public void periodic() {
        Dashboard.getInstance().balance.setAngle(getPitch());

        /*
         * If the absolute value of the robot's pitch is within 2.0 degrees, show the
         * robot as balanced on the dashboard
         * Will be useful for evaluating if manual adjustments are needed
         */
        if (Math.abs(getPitch()) < 2.0) {
            Dashboard.getInstance().balance.setBalanced(true);
        } else {
            Dashboard.getInstance().balance.setBalanced(false);
        }
    }
}
