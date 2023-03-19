package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    public static enum IntakeState {
        /** Cubes & also helps grab cones */
        OPEN,
        /** Cones only */
        CLOSED;
    }

    DoubleSolenoid intakeActuator;
    CANSparkMax intakeMotor;

    private IntakeSubsystem(){
        intakeActuator = PneumaticHubSubsystem.getDoubleSolenoid(IntakeConstants.INTAKE_OPEN_CLOSE);

        intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_DRIVE_CAN_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public synchronized void openIntake(){
        intakeActuator.set(DoubleSolenoid.Value.kForward);
    }

    public synchronized void closeIntake(){
        intakeActuator.set(DoubleSolenoid.Value.kReverse);
    }

    public void runIntakeDrive(double speed) {
        intakeMotor.set(speed);
    }

    public synchronized void switchIntake(){
        if(intakeActuator.get() == DoubleSolenoid.Value.kForward) {       
            openIntake();
        } else if (intakeActuator.get() == DoubleSolenoid.Value.kReverse) {
            closeIntake();
        }
    }


}
