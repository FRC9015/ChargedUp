package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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

    /**
     * As compared to {@link #getSlowed()}, this method will not return true if the robot is in autonomous mode.
     */
    public static boolean getSlowedSmart() {
        return runningSlow && !(edu.wpi.first.wpilibj.RobotState.isAutonomous());
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

    private static Pose2d savedpoint = new Pose2d();

    public static Pose2d getSavedPoint(){
        return savedpoint;
    }

    public static void setSavedPoint(Pose2d newPoint){
        savedpoint = newPoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotState");
        builder.addBooleanProperty("runningSlow", RobotState::getSlowed, this::setSlowed);
    }
}