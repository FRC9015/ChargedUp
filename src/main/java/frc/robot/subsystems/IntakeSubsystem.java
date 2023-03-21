package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null)
            INSTANCE = new IntakeSubsystem();
        return INSTANCE;
    }

    /**
     * Represents state of the intake pneumatics
     */
    public static enum IntakeState {
        /** Cubes & also helps grab cones */
        OPEN,
        /** Cones only */
        CLOSED;
    }

    DoubleSolenoid intakeActuator;
    CANSparkMax intakeMotor;

    Timer controlFrameTimer = new Timer();

    private IntakeSubsystem() {
        intakeActuator = PneumaticHubSubsystem.getDoubleSolenoid(IntakeConstants.INTAKE_OPEN_CLOSE);

        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_DRIVE_CAN_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        /* If the intake drive wheel speed hasn't been updated in at least 500ms,
         * report a warning and stop the drive wheels
         */
        if (controlFrameTimer.hasElapsed(0.5)) {
            DriverStation.reportWarning("INTAKE SUBSYSTEM: Drive Not Updated Frequently Enough", false);
            runIntakeDrive(0);
        }
    }

    public void openIntake() {
        synchronized (intakeActuator) {
            intakeActuator.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void closeIntake() {
        synchronized (intakeActuator) {
            intakeActuator.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void runIntakeDrive(double speed) {
        controlFrameTimer.restart();
        intakeMotor.set(speed);
    }

    public void switchIntake() {
        synchronized (intakeActuator) {
            if (intakeActuator.get() == DoubleSolenoid.Value.kForward) {
                openIntake();
            } else if (intakeActuator.get() == DoubleSolenoid.Value.kReverse) {
                closeIntake();
            }
        }
    }

}
