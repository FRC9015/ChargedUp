package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import lombok.Synchronized;

public class RobotState implements Sendable {
    
    private static RobotState INSTANCE;

    public static RobotState getInstance() {
        if(INSTANCE == null) INSTANCE = new RobotState();
        return INSTANCE;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotState");
        builder.addBooleanProperty("runningSlow", RobotState::getSlowed, this::setSlowed);
    }

    /* ---------- Drivetrain Slowed State ---------- */

    private static Object $lockSlow = new Object();
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

    @Synchronized("$lockSlow")
    public static void setSlow() {
        runningSlow = true;
    }

    @Synchronized("$lockSlow")
    public static void setFast() {
        runningSlow = false;
    }

    @Synchronized("$lockSlow")
    public static boolean toggleSlow() {
        runningSlow = !runningSlow;
        return runningSlow;
    }

    @Synchronized("$lockSlow")
    private void setSlowed(boolean isSlowed) {
        runningSlow = isSlowed;
    }

    /* ---------- Saved Waypoint State ---------- */

    private static Object $lockWaypoint = new Object();
    private static Pose2d savedWaypoint = new Pose2d();

    public static Pose2d getSavedPoint(){
        return savedWaypoint;
    }

    @Synchronized("$lockWaypoint")
    public static void setSavedPoint(Pose2d newPoint){
        savedWaypoint = newPoint;
    }
}