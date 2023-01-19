package frc.robot;

public class RobotState {
    
    private final static RobotState INSTANCE = new RobotState();

    public static RobotState getInstance() {
        return INSTANCE;
    }

    private static boolean runningSlow = false;

    public static boolean getSlowed() {
        return runningSlow;
    }

    public static synchronized void setSlow() {
        runningSlow = true;
    }

    public static synchronized void setFast() {
        runningSlow = false;
    }

    public static synchronized boolean toggleSlow() {
        runningSlow = !runningSlow;

        return runningSlow;
    }
}