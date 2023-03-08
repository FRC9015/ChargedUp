package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class BlinkinSubsystem implements Subsystem{
  
  private final static BlinkinSubsystem INSTANCE = new BlinkinSubsystem(1,1,1); // The 1s are placeholder values.

  @SuppressWarnings("WeakerAccess")
  public static BlinkinSubsystem getInstance() {
    return INSTANCE;
  }
  
  // Variables
  private final CANSparkMax LED;
  private final Servo ledServo;
  // private final int blinkinChannel;
  
  
  // Creates a new BlinkinSubsystem.
  public BlinkinSubsystem(int motorId, int blinkinChannel, int servoId) 
  {
    // Connect to Blinkin and set ServoID.
      LED = new CANSparkMax(motorId, MotorType.kBrushless);
      LED.setIdleMode(IdleMode.kCoast);
      ledServo = new Servo(servoId);
      // this.blinkinChannel = blinkinChannel;
  }

  
  
  // Methods
  public void setLED(double value) {
    ledServo.set(value);
  }

  public void setTeamColors(){
    setLED(1695); // Color 2 (Orange) sparkles on color 1 (Blue). The colors must be manually set on the Rev Blinkin.
  }
  public void setCone(){
    setLED(1845); // Sets the color to yellow for the cone.
  }

  public void setCube(){
    setLED(1995); // Sets the color to violet for the cube.
  }
}