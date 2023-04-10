package frc.robot.subsystems;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.qefrc.qelib.led.LEDPattern;
import frc.qefrc.qelib.led.LEDSection;
import frc.qefrc.qelib.led.LEDSectionController;
import frc.qefrc.qelib.led.LEDStrip;
import frc.qefrc.qelib.led.patterns.QERainbow;

import frc.robot.Helpers;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;

    public static LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }

    private final int LED_PORT = 0;
    private final int LED_LENGTH = 139;

    public final Color QE_NAVYBLUE = new Color(17, 43, 60);
    public final Color QE_BLUE = new Color(32, 83, 117);
    public final Color QE_ORANGE = new Color(255, 126, 14);
    public final Color CONE = Color.kYellow;
    public final Color CUBE = Color.kPurple;

    private AddressableLEDBuffer ledBuffer;
    private LEDStrip customLED;
    private LEDPattern qerainbow;
    private LEDSection defaultSection, halfSection;

    // Effect Variables
    private int rainbowFirstPixelHue;
    private int chaseOffset;
    private int scannerEyePosition = 0;
    private int scannerScanDirection = 1;

    private LEDSubsystem() {

        customLED = new LEDStrip(LED_PORT, LED_LENGTH);
        defaultSection = customLED.getSection(0, LED_LENGTH - 1);
        halfSection = customLED.getSection(0, 60);
        qerainbow =
                new QERainbow(halfSection);

        customLED.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            customLED.setColor(Color.kRed);
        } else {
            customLED.setPattern(qerainbow, true);
        }
        customLED.update();
    }

    /* ---------- BASE LED METHODS ---------- */

    /**
     * Set a single static color across the LED strip
     *
     * @param color The color to set
     */
    public void staticColor(Color color) {
        synchronized (ledBuffer) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, color);
            }
        }
    }

    public void pulsetwoColor(Color color1, Color color2, double frequency, double another) {
        synchronized (ledBuffer) {
            double time = System.currentTimeMillis() / 1000.0; // current time in seconds
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                double brightness =
                        0.5
                                + 0.5
                                        * Math.sin(
                                                2 * Math.PI * frequency * i / ledBuffer.getLength()
                                                        + time * another);
                // brightness *= amplitude;

                int red = (int) ((color1.red * brightness + color2.red * (1 - brightness)) * 255);
                int green =
                        (int) ((color1.green * brightness + color2.green * (1 - brightness)) * 255);
                int blue =
                        (int) ((color1.blue * brightness + color2.blue * (1 - brightness)) * 255);

                ledBuffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    /**
     * Set a color within a range of LEDs
     *
     * @param start first led to set
     * @param end last led to set
     * @param color color to set
     */
    public void staticColorInRange(int start, int end, Color color) {
        start = MathUtil.clamp(start, 0, ledBuffer.getLength());
        end = MathUtil.clamp(end, 0, ledBuffer.getLength());

        synchronized (ledBuffer) {
            for (int i = start; i < end; i++) {
                ledBuffer.setLED(i, color);
            }
        }
    }

    /* --------- BASIC COLORS --------- */

    public void red() {
        staticColor(Color.kRed);
    }

    public void red(int start, int end) {
        staticColorInRange(start, end, Color.kRed);
    }

    public void blue() {
        staticColor(Color.kBlue);
    }

    public void blue(int start, int end) {
        staticColorInRange(start, end, Color.kBlue);
    }

    /* -------- EFFECTS ---------- */

    public void rainbow() {
        synchronized (ledBuffer) {
            // For every pixel
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                // Set the value
                ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            rainbowFirstPixelHue += 3;
            // Check bounds
            rainbowFirstPixelHue %= 180;
        }
    }

    public void pulseColor(Color color, double frequency, double amplitude, double another) {
        synchronized (ledBuffer) {
            double time = System.currentTimeMillis() / 1000.0; // current time in seconds
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                double brightness =
                        0.5
                                + 0.5
                                        * Math.sin(
                                                2 * Math.PI * frequency * i / ledBuffer.getLength()
                                                        + time * another);
                brightness *= amplitude;
                int red = (int) (color.red * brightness);
                int green = (int) (color.green * brightness);
                int blue = (int) (color.blue * brightness);
                ledBuffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    public void pulseColorSolid(Color color, double frequency, double amplitude, double another) {
        synchronized (ledBuffer) {
            double time = System.currentTimeMillis() / 1000.0; // current time in seconds
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                double brightness =
                        0.5
                                + 0.5
                                        * Math.sin(
                                                2 * Math.PI * frequency / ledBuffer.getLength()
                                                        + time * another);
                brightness *= amplitude;
                int red = (int) (color.red * brightness);
                int green = (int) (color.green * brightness);
                int blue = (int) (color.blue * brightness);
                ledBuffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    public void pulseTwoColor(Color color1, Color color2, double frequency, double another) {
        synchronized (ledBuffer) {
            double time = System.currentTimeMillis() / 1000.0; // current time in seconds
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                double brightness =
                        0.5
                                + 0.5
                                        * Math.sin(
                                                2 * Math.PI * frequency * i / ledBuffer.getLength()
                                                        + time * another);
                // brightness *= amplitude;

                int red = (int) ((color1.red * brightness + color2.red * (1 - brightness)) * 255);
                int green =
                        (int) ((color1.green * brightness + (color2.green * 1 - brightness)) * 255);
                int blue =
                        (int) ((color1.blue * brightness + (color2.blue * 1 - brightness)) * 255);

                ledBuffer.setLED(i, new Color(red, green, blue));
            }
        }
    }

    /**
     * Color Chasing Pattern; segments of each color "chase" down the led strip
     *
     * @param colors array of colors to chase with
     * @param segmentWidth number of leds that should be filled with each color
     */
    public void chaseColors(Color[] colors, int segmentWidth) {
        int numberOfColors = colors.length;
        int effectiveIndex;
        int colorIndex;
        int bufferLength = ledBuffer.getLength();
        synchronized (ledBuffer) {
            for (int index = 0; index < bufferLength; index++) {
                effectiveIndex = (index + chaseOffset) % bufferLength;
                colorIndex = (index / segmentWidth) % numberOfColors;
                ledBuffer.setLED(effectiveIndex, colors[colorIndex]);
            }
        }

        chaseOffset = (chaseOffset + 1) % bufferLength;
    }

    /**
     * @param lowColor
     * @param highColor
     * @param intensity number in range 0 -> 1 where 0 is the low color and 1 is the high color
     */
    public void intensityPattern(Color lowColor, Color highColor, double intensity) {
        intensity = MathUtil.clamp(intensity, 0, 1);
        double red = MathUtil.interpolate(lowColor.red, highColor.red, intensity);
        double green = MathUtil.interpolate(lowColor.green, highColor.green, intensity);
        double blue = MathUtil.interpolate(lowColor.blue, highColor.blue, intensity);
        synchronized (ledBuffer) {
            for (int index = 0; index < ledBuffer.getLength(); index++) {
                ledBuffer.setLED(index, new Color(red, green, blue));
            }
        }
    }

    /**
     * Color Scanning effect; the "eye" color scans back and forth. This is different than "chase"
     * because the eye fades in and out over it's length
     *
     * @param background the color that fills the rest of the LED strip
     * @param eye the color that scans back and forth
     * @param length the length of the scanner
     */
    public void scanner(Color background, Color eye, int length) {
        int bufferLength = ledBuffer.getLength();
        double intensity;
        double red;
        double green;
        double blue;
        double distanceFromEye;

        synchronized (ledBuffer) {
            for (int index = 0; index < bufferLength; index++) {
                distanceFromEye = MathUtil.clamp(Math.abs(scannerEyePosition - index), 0, length);
                intensity = 1 - distanceFromEye / length;
                red = MathUtil.interpolate(background.red, eye.red, intensity);
                green = MathUtil.interpolate(background.green, eye.green, intensity);
                blue = MathUtil.interpolate(background.blue, eye.blue, intensity);

                ledBuffer.setLED(index, new Color(red, green, blue));
            }
        }

        if (scannerEyePosition == 0) {
            scannerScanDirection = 1;
        } else if (scannerEyePosition == bufferLength - 1) {
            scannerScanDirection = -1;
        }

        scannerEyePosition += scannerScanDirection;
    }

    /**
     * Pure color chaos. Random colors on each pixel.
     *
     * @param firstTime
     */
    public void chaos(boolean firstTime) {
        // This function randomly regenerates a color
        Function<Color, Color> randomColorShift =
                (Color aColor) ->
                        new Color(
                                Helpers.randomShift(aColor.red),
                                Helpers.randomShift(aColor.green),
                                Helpers.randomShift(aColor.blue));

        synchronized (ledBuffer) {
            if (firstTime) {
                for (int index = 0; index < ledBuffer.getLength(); index++) {
                    ledBuffer.setLED(index, new Color(Math.random(), Math.random(), Math.random()));
                }
                firstTime = false;
            }
            for (int index = 0; index < ledBuffer.getLength(); index++) {
                ledBuffer.setLED(index, randomColorShift.apply(ledBuffer.getLED(index)));
            }
        }
    }

    /**
     * Alternates colors in the given list as it goes down the line. This pattern does not move.
     *
     * @param colors list of colors to alternate between
     */
    public void alternateStatic(Color[] colors) {
        synchronized (ledBuffer) {
            for (int index = 0; index < ledBuffer.getLength(); index++) {
                ledBuffer.setLED(index, colors[index % colors.length]);
            }
        }
    }

    /* ---------- HIGH LEVEL METHODS ---------- */

    public void pulseTeamColors() {
        pulseTwoColor(Color.kBlue, QE_ORANGE, 0.5, 2);
    }
}
