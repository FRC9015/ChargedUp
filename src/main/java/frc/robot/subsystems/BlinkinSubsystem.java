package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class BlinkinSubsystem implements Subsystem{
  private final CANSparkMax LED;
  private final Servo ledServo;
  
  // Connect to Blinkin and set ServoID
  public BlinkinSubsystem(int motorId, int servoId) {
    LED = new CANSparkMax(motorId, MotorType.kBrushless);
    LED.setIdleMode(IdleMode.kCoast);
    ledServo = new Servo(servoId);
   
  }
  
  public void setLED(double value) {
    ledServo.set(value);
  }

  public void setTeamColors(){
    setLED(1005); // This value will be changed.
  }
  public void setCone(){
    setLED(1005); // But rainbows are cool
  }

  public void setCube(){
    setLED(1005); // :)
  }
  
}