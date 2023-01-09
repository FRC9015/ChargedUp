package frc.robot;

public class RobotState {
    
    private final static RobotState INSTANCE = new RobotState();

    public static RobotState getInstance() {
        return INSTANCE;
    }

    private boolean runningSlow = false;

    public boolean getSlowed() {
        return runningSlow;
    }

    public void setSlow() {
        runningSlow = true;
    }

    public void setFast() {
        runningSlow = false;
    }

    public boolean toggleSlow() {
        runningSlow = !runningSlow;

        return runningSlow;
    }
}
