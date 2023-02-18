package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsytem extends SubsystemBase {


    private final static LimelightSubsytem INSTANCE = new LimelightSubsytem();

    NetworkTable limelight;
    NetworkTableEntry tx, ty, ta, tv, tid, botpose;

    public static enum CamMode {
        VISION, // Vision processor
        DRIVER  // Driver Camera (Increases exposure, disables vision processing)
    }

    @SuppressWarnings("WeakerAccess")
    public static LimelightSubsytem getInstance() {
        return INSTANCE;
    }

    public LimelightSubsytem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
        ta = limelight.getEntry("ta");
        tv = limelight.getEntry("tv");
        tid = limelight.getEntry("tid");
        botpose = limelight.getEntry("botpose");

    }

    /** 
     * Check if the camera has (a) target(s) in sight
     */
    public boolean hasTargets() {
        double tvResult = tv.getDouble(0);
        if (tvResult == 0) {
            return false;
        } else {
            return true;
        }
    }

    /** 
     * Get the ID of the detected primary AprilTag
     */
    public int getPrimaryAprilTag() {
        double rawTagId = tid.getDouble(-1);
        return (int) rawTagId;
    }

    /** 
     * Set the mode of the camera
     */
    public void setMode(CamMode newMode) {
        NetworkTableEntry numMode = limelight.getEntry("camMode");
        if (newMode == CamMode.VISION) {
            numMode.setNumber(0);
        } else if (newMode == CamMode.DRIVER) {
            numMode.setNumber(1);
        }
    }
    public double getTx(){
        return tx.getDouble(0.0);
    }

    public double[] getBotpose(){
        if (hasTargets()){
        return botpose.getDoubleArray(new double[6]);}
        else{
            
            return new double[6];
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

}