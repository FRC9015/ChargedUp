package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class RobotState implements Sendable {
    
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

    private synchronized void setSlowed(boolean isSlowed) {
        runningSlow = isSlowed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotState");
        builder.addBooleanProperty("runningSlow", RobotState::getSlowed, this::setSlowed);
    }
}