package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederIntakeConstants;
import frc.robot.RobotState;

public class FeederIntakeSubsystem extends SubsystemBase {

    private static FeederIntakeSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static FeederIntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new FeederIntakeSubsystem();
        return INSTANCE;
    }

    private boolean forward;

    CANSparkMax feederIntakeMotor;
    DoubleSolenoid feederIntake;

    private FeederIntakeSubsystem() {
        feederIntakeMotor = new CANSparkMax(FeederIntakeConstants.FEEDER_INTAKE_DRIVE_CAN_ID, MotorType.kBrushless);
        feederIntakeMotor.setSmartCurrentLimit(20, 30);
        feederIntake = PneumaticHubSubsystem.getDoubleSolenoid(FeederIntakeConstants.OPEN_CLOSE_SOLENOID);
        forward = false;
    }

    @Override
    public void periodic() {
        RobotState.setFeederIntakeOpen(!(feederIntake.get() == DoubleSolenoid.Value.kReverse));
    }

    public void extendFeeder() {
        feederIntake.set(DoubleSolenoid.Value.kForward);
    }

    public void retractFeeder() {
        feederIntake.set(DoubleSolenoid.Value.kReverse);
    }

    public void setIntakeMotorSpeed(double speed) {
        feederIntakeMotor.set(speed);
    }

    public void switchFeeder() {
        if (forward) {
            feederIntake.set(DoubleSolenoid.Value.kReverse);
            forward = false;

        } else {
            feederIntake.set(DoubleSolenoid.Value.kForward);
            forward = true;
        }
    }
}
