package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.utils.Blinkin.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * Subsystem for interfacing with the REV Robotics Blinkin LED Driver
 */
public class BlinkinSubsystem implements Subsystem {

  private static BlinkinSubsystem INSTANCE;

  @SuppressWarnings("WeakerAccess")
  public static BlinkinSubsystem getInstance() {
    if (INSTANCE == null)
      INSTANCE = new BlinkinSubsystem();
    return INSTANCE;
  }

  /*
   * A Spark motor controller instance is used to control the Blinkin
   * See page 7, section 2.3:
   * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  private final Spark blinkin;
  private double currentValue = SolidColor.Off.val;

  private BlinkinSubsystem() {
    blinkin = new Spark(LEDConstants.BLINKIN_PWM_PORT);
  }

  /* ---------- ABSTRACTED COMMON FUNCTIONALITY ---------- */

  public void setCube() {
    setColor(SolidColor.BlueViolet);
  }

  public void setCone() {
    setColor(SolidColor.Yellow);
  }

  /* ---------- LOWER LEVEL CONTROL METHODS ---------- */

  /**
   * Set a pattern using one of the fixed {@link frc.robot.utils.Blinkin.ColorScheme ColorScheme}'s and {@link frc.robot.utils.Blinkin.FixedPatternType fixed pattern}s.
   * @param scheme {@link frc.robot.utils.Blinkin.ColorScheme ColorScheme} to use; Rainbow, Party, Ocean, Lava, or Forest 
   * @param pattern {@link frc.robot.utils.Blinkin.FixedPatternType Pattern}; Rainbow, Sinelon, Beats Per Minute, Twinkles, or Color Wave
   */
  public void setPattern(ColorScheme scheme, FixedPatternType pattern) {
    setLED(scheme.val + pattern.val);
  }

  /**
   * Set a pattern using a 
   * @param color Programmed color to use; Color 1 or Color 2
   * @param pattern Pattern to use with color
   */
  public void setPattern(Color color, CustomPatternType pattern) {
    setLED(color.val + pattern.val);
  }

  /**
   * Display a pattern using both programmed colors
   * @param pattern Combination pattern to use
   */
  public void setPattern(CustomCombiPattern pattern) {
    setLED(pattern.val);
  }

  /**
   * Display one of the built-in solid colors
   * @param color Built In Color to set
   */
  public void setColor(SolidColor color) {
    setLED(color.val);
  }

  /**
   * Private method that limits decimal places to 2 for precision and sets them on
   * @param num
   */
  private void setLED(double num) {
    currentValue = new BigDecimal(num).setScale(2, RoundingMode.HALF_UP).doubleValue();
    synchronized(blinkin) {
      blinkin.set(currentValue);
    }
  }
}