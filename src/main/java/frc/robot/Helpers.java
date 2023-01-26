package frc.robot;

public class Helpers {

    /**
     * Constrain a number within the limit, positive or negative
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
}
