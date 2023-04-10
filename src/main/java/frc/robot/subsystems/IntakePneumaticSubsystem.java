package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;

public class IntakePneumaticSubsystem extends SubsystemBase {

    private static IntakePneumaticSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static IntakePneumaticSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakePneumaticSubsystem();
        return INSTANCE;
    }

    private boolean forward;

    CANSparkMax intakeMotor;
    DoubleSolenoid intake;
    MedianFilter intakeCurrentFilter;
    double intakeCurrent;

    private IntakePneumaticSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_DRIVE_CAN_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(20, 30);
        intake = PneumaticHubSubsystem.getDoubleSolenoid(IntakeConstants.OPEN_CLOSE_SOLENOID);
        intakeCurrentFilter = new MedianFilter(25);
        forward = false;
    }

    @Override
    public void periodic() {
        RobotState.setIntakeOpen(!(intake.get() == DoubleSolenoid.Value.kReverse));
        intakeCurrent = intakeCurrentFilter.calculate(intakeMotor.getOutputCurrent());

        if (Math.abs(intakeMotor.get()) > 0) {
            if (intakeCurrent > 15) {
                System.out.println("INTAKE CUBE IN");
            } else {
                System.out.println("INTAKE CUBE OUT");
            }
        }
    }

    public void openIntake() {
        intake.set(DoubleSolenoid.Value.kReverse);
    }

    public void closeIntake() {
        intake.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntakeMotorSpeed(double speed) {
        intakeMotor.set(intakeCurrent > 15 ? speed * 0.5 : speed);
    }

    public void switchIntake() {
        if (forward) {
            intake.set(DoubleSolenoid.Value.kReverse);
            forward = false;

        } else {
            intake.set(DoubleSolenoid.Value.kForward);
            forward = true;
        }
    }
}
