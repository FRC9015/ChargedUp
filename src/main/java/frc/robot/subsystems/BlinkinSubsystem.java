package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class BlinkinSubsystem implements Subsystem{
  private final CANSparkMax LED;
  private final Servo ledServo;
 
  
  private final int blinkinChannel;
  private final double MinLEDPulseWidth;
  private final double MaxLEDPulseWidth;
  
  //PID 
  public BlinkinSubsystem(int motorId, int blinkinChannel, int servoId, double MinLEDPulseWidth, double MaxLEDPulseWidth) {
    LED = new CANSparkMax(motorId, MotorType.kBrushless);
    LED.setIdleMode(IdleMode.kCoast);
    
    ledServo = new Servo(servoId);
    this.blinkinChannel = blinkinChannel;
    this.MinLEDPulseWidth = MinLEDPulseWidth; 
    this.MaxLEDPulseWidth = MaxLEDPulseWidth;
  }
  
  public void setLED(double value) {
    ledServo.set(value);
  }

  public void setTeamColors(){
    setLED(1005); //This value will be changed.
  }
  public void setCone(){
    setLED(1005); //But rainbows are cool
  }

  public void setCube(){
    setLED(1005); //:)
  }
  
}