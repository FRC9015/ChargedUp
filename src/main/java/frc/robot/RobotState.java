package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class RobotState implements Sendable {

    private static final RobotState INSTANCE = new RobotState();

    public static RobotState getInstance() {
        return INSTANCE;
    }

    private static boolean runningSlow = false;

    public static boolean getSlowed() {
        return runningSlow;
    }

    /**
     * As compared to {@link #getSlowed()}, this method will not return true if the robot is in
     * autonomous mode.
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

    private static boolean intakeOpen = false;
    private static boolean feederIntakeOpen = false;

    public static boolean getIntakeOpen() {
        return intakeOpen;
    }

    public static boolean getFeederIntakeOpen() {
        return feederIntakeOpen;
    }

    public static synchronized void setIntakeOpen(boolean isOpen) {
        intakeOpen = isOpen;
    }

    public static synchronized void setFeederIntakeOpen(boolean isOpen) {
        feederIntakeOpen = isOpen;
    }

    // BEGIN Piston Extension
    private static boolean pistonExtended = false;

    public static boolean getPistonExtended() {
        return pistonExtended;
    }

    public static synchronized void setPistonExtended(boolean isExtended) {
        pistonExtended = isExtended;
    }
    // END Piston Extension

    private static boolean footDown = false;

    public static boolean isFeetDown() {
        return footDown;
    }

    public static synchronized void setFeetDown(boolean isDown) {
        footDown = isDown;
    }

    private static Pose2d savedpoint = new Pose2d();

    public static Pose2d getSavedPoint() {
        return savedpoint;
    }

    public static void setSavedPoint(Pose2d newPoint) {
        savedpoint = newPoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotState");
        builder.addBooleanProperty("runningSlow", RobotState::getSlowed, this::setSlowed);
    }
}
