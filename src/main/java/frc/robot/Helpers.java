package frc.robot;

import lombok.experimental.UtilityClass;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;

@UtilityClass
public class Helpers {

    /**
     * Constrain a number within the limit, positive or negative
     *
     * @param input Input to test against limit
     * @param limit Positive, nonzero number
     * @return
     */
    public static double limitDecimal(double input, double limit) {
        limit = Math.abs(limit); // Just in case a negative is passed in
        double result = 0;

        if (Math.abs(input) > limit) { // Test if the absolute value of the input exceeds the limit
            if (input > 0) { // Return positive limit
                result = limit;
            } else if (input < 0) { // Return negative limit
                result = -limit;
            }
        } else {
            result = input;
        }

        return result;
    }

    public double calcDeadzone(double input, double dz) {
        if (Math.abs(input) <= dz) {
            return 0;
        } else {
            return input;
        }
    }

    public static void logBox(String... msgs) {
        int maxLength = Integer.MIN_VALUE;

        for (String msg : msgs) {
            if (maxLength < msg.length()) {
                maxLength = msg.length();
            }
        }

        maxLength += 4;

        String capString = "";

        for (int i = 0; i <= maxLength; i++) {
            capString += "*";
        }

        System.out.println();
        System.out.println(capString);

        for (String msg : msgs) {
            System.out.println("* " + msg);
        }

        System.out.println(capString);
        System.out.println();
    }

    public static void handleRevError(REVLibError status, String yay) {
        if (status == REVLibError.kOk) {
            Helpers.logBox("YAY: " + yay);
        } else {
            throw new Error("FOLLOW ERROR: " + status.toString());
        }
    }

    /**
     * Generate a random value within a given value
     *
     * @param value any value
     * @return a random percentage of the given value
     */
    public static double randomShift(double value) {
        double sign = Math.random() >= 0.5 ? 1.0 : -1.0;
        double amount = Math.random() / 10;
        return MathUtil.clamp(value + sign * amount, 0, 1);
    }
}
