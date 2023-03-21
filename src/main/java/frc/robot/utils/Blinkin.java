package frc.robot.utils;

/**
   * Enums that represent all the different color/pattern states of a Blinkin. <br></br>
   * See page 14, the LED Pattern Table: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   * 
   */
public final class Blinkin {
    /**
     * Blinkin built-in Color Schemes. Used in conjuction with {@link FixedPatternType}
     */
    public static enum ColorScheme {
        Rainbow(-0.09),
        Party(-0.07),
        Ocean(-0.05),
        Lave(-0.03),
        Forest(-0.01);

        public final double val;

        private ColorScheme(double val) {
            this.val = val;
        }
    }

    /**
     * Patterns that use built-in color schemes. Used in conjuction with {@link ColorScheme}
     */
    public static enum FixedPatternType {
        Rainbow(-0.9),
        Sinelon(-0.7),
        BeatsPerMinute(-0.6),
        Twinkles(-0.54),
        ColorWaves(-0.44);

        public final double val;

        private FixedPatternType(double val) {
            this.val = val;
        }
    }

    /**
     * Select which custom color to use. Used in conjuction with {@link CustomPatternType}
     */
    public static enum Colors {
        Color1(0.0),
        Color2(0.20);

        public final double val;

        private Colors(double val) {
            this.val = val;
        }
    }
    
    /**
     * Patterns that use one custom color. Used in conjuction with {@link Colors}
     */
    public static enum CustomPatternType {
        E2EBlendBlack(-0.03),
        LarsonScanner(-0.01),
        LightChase(0.01),
        HeartbeatSlow(0.03),
        HeartbeatMed(0.05),
        HeartbeatFast(0.07),
        BreathSlow(0.09),
        BreathFast(0.11),
        Shot(0.13),
        Strobe(0.15);

        public final double val;

        private CustomPatternType(double val) {
            this.val = val;
        }
    }

    /**
     * Patterns that combine colors 1 and 2.
     */
    public static enum CustomCombiPattern {
        /** Sparkle, Color 1 on Color 2 */
        Sparkle_1on2(0.37),
        /** Sparkle, Color 2 on Color 1 */
        Sparkle_2on1(0.39),
        /** Color Gradient, Color 1 and 2 */
        Gradient(0.41),
        /** Beats Per Minute, Color 1 and 2 */
        BeatsPerMinute(0.43),
        /** End to End Blend, Color 1 to 2 */
        EndToEndBlend_1to2(0.45),
        /** End to End Blend */
        EndToEndBlend(0.47),
        /**
         * Color 1 and Color 2 no blending <br>
         * </br>
         * <b>Setup Pattern</b>
         */
        Color1and2(0.49),
        SetupPattern(0.49),
        /** Twinkles, Color 1 and 2 */
        Twinkles(0.51),
        /** Color Waves, Color 1 and 2 */
        ColorWaves(0.53),
        /** Sinelon, Color 1 and 2 */
        Sinelon(0.55);

        public final double val;

        private CustomCombiPattern(double val) {
            this.val = val;
        }
    }

    /**
     * Built in solid colors.
     */
    public static enum SolidColor {
        HotPink(0.57),
        DarkRed(0.59),
        Red(0.61),
        RedOrange(0.63),
        Orange(0.65),
        Gold(0.67),
        Yellow(0.69),
        LawnGreen(0.71),
        Lime(0.73),
        DarkGreen(0.75),
        Green(0.77),
        BlueGreen(0.79),
        Aqua(0.81),
        SkyBlue(0.83),
        DarkBlue(0.85),
        Blue(0.87),
        BlueViolet(0.89),
        Violet(0.91),
        White(0.93),
        Gray(0.95),
        DarkGray(0.97),
        Black(0.99),
        Off(0.99);

        public final double val;

        private SolidColor(double val) {
            this.val = val;
        }
    }
}
