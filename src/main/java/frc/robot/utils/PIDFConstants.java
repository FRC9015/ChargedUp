package frc.robot.utils;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class that represents PIDF constants, and allows editing of them via Sendable.
 */
public class PIDFConstants implements Sendable {
    private double kP, kI, kD, kIZone, kFF;

    /**
     * Construct with only basic PID constants
     * @param P Proportional term
     * @param I Integral term
     * @param D Derivative term
     */
    public PIDFConstants(double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

    /**
     * Construct constants for a PID controller with FeedFoward
     * @param P Proportional term
     * @param I Integral term
     * @param D Derivative term
     * @param IZone Specifies the range the |error| must be within for the integral term to take effect. Must be positive, or set to zero to disable.
     * @param FF FeedForward term
     */
    public PIDFConstants(double P, double I, double D, double IZone, double FF) {
        kP = P;
        kI = I;
        kD = D;
        kIZone = IZone;
        kFF = FF;
    }

    /**
     * Construct PIDF constants from a SparkMaxPIDController
     * @param basePID {@link SparkMaxPIDController} that we should inherit initial values from
     * @param slotID PID Slot to read
     */
    public PIDFConstants(SparkMaxPIDController basePID) {
        kP = basePID.getP();
        kI = basePID.getI();
        kD = basePID.getD();
        kIZone = basePID.getIZone();
        kFF = basePID.getFF();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(this.getClass().getName());
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("IZone", this::getIZone, this::setIZone);
        builder.addDoubleProperty("FF", this::getFF, this::setFF);
    }

    /**
     * Update a SparkMaxPIDController with the values in this object. Useful for tuning from the dashboard. <p>
     * <b>IMPORTANT:</b> These settings will not persist unless the {@link com.revrobotics.CANSparkMax#burnFlash() CANSparkMax.burnFlash()} method is called.
     * @param toUpdate {@link SparkMaxPIDController} to update  
     * @param slotID PID Slot to update
     */
    public synchronized void updateSparkMax(SparkMaxPIDController toUpdate, int slotID) {
        // Method is synchronized to prevent simultaneous updates
        if (toUpdate.getP() != kP) toUpdate.setP(kP, slotID);
        if (toUpdate.getI() != kI) toUpdate.setI(kI, slotID);
        if (toUpdate.getD() != kD) toUpdate.setD(kD, slotID);
        if (toUpdate.getIZone() != kIZone) toUpdate.setIZone(kIZone, slotID);
        if (toUpdate.getFF() != kFF) toUpdate.setFF(kFF, slotID);
    }

    /**
     * Update a SparkMaxPIDController with the values in this object. Useful for tuning from the dashboard. <p>
     * <b>IMPORTANT:</b> These settings will not persist unless the {@link com.revrobotics.CANSparkMax#burnFlash() CANSparkMax.burnFlash()} method is called.
     * @param toUpdate {@link SparkMaxPIDController} to update  
     */
    public synchronized void updateSparkMax(SparkMaxPIDController toUpdate) {
        // Method is synchronized to prevent simultaneous updates
        if (toUpdate.getP() != kP) toUpdate.setP(kP);
        if (toUpdate.getI() != kI) toUpdate.setI(kI);
        if (toUpdate.getD() != kD) toUpdate.setD(kD);
        if (toUpdate.getIZone() != kIZone) toUpdate.setIZone(kIZone);
        if (toUpdate.getFF() != kFF) toUpdate.setFF(kFF);
    }

    public double getP() {
        return kP;
    }

    public void setP(double P) {
        this.kP = P;
    }

    public double getI() {
        return kI;
    }

    public void setI(double I) {
        this.kI = I;
    }

    public double getD() {
        return kD;
    }

    public void setD(double D) {
        this.kD = D;
    }

    public double getIZone() {
        return kIZone;
    }

    public void setIZone(double IZone) {
        this.kIZone = IZone;
    }

    public double getFF() {
        return kFF;
    }

    public void setFF(double FF) {
        this.kFF = FF;
    }
}
