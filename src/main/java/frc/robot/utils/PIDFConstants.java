package frc.robot.utils;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Class that represents PIDF constants, and allows editing of them via
 * Sendable.
 */
public class PIDFConstants implements Sendable {
    private double kP, kI, kD, kIZone, kFF;
    private String dashName;

    /**
     * Construct with only basic PID constants
     * 
     * @param P Proportional term
     * @param I Integral term
     * @param D Derivative term
     */
    public PIDFConstants(double P, double I, double D) {
        this(P, I, D, null);
    }

    /**
     * Construct with only basic PID constants
     * 
     * @param P Proportional term
     * @param I Integral term
     * @param D Derivative term
     */
    public PIDFConstants(double P, double I, double D, String name) {
        kP = P;
        kI = I;
        kD = D;
        dashName = name;
    }

    /**
     * Construct constants for a PID controller with FeedFoward
     * 
     * @param P     Proportional term
     * @param I     Integral term
     * @param D     Derivative term
     * @param IZone Specifies the range the |error| must be within for the integral
     *              term to take effect. Must be positive, or set to zero to
     *              disable.
     * @param FF    FeedForward term
     */
    public PIDFConstants(double P, double I, double D, double IZone, double FF) {
        this(P, I, D, IZone, FF, null);
    }

    /**
     * Construct constants for a PID controller with FeedFoward
     * 
     * @param P     Proportional term
     * @param I     Integral term
     * @param D     Derivative term
     * @param IZone Specifies the range the |error| must be within for the integral
     *              term to take effect. Must be positive, or set to zero to
     *              disable.
     * @param FF    FeedForward term
     * @param name  Name that should be shown on the dashboard
     */
    public PIDFConstants(double P, double I, double D, double IZone, double FF, String name) {
        kP = P;
        kI = I;
        kD = D;
        kIZone = IZone;
        kFF = FF;

        dashName = name;
    }

    /**
     * Construct PIDF constants from a SparkMaxPIDController
     * 
     * @param basePID {@link SparkMaxPIDController} that we should inherit initial
     *                values from
     * @param slotID  PID Slot to read
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
        builder.setSmartDashboardType(dashName == null ? this.getClass().getName() : dashName);
        builder.setActuator(true);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("IZone", this::getIZone, this::setIZone);
        builder.addDoubleProperty("FF", this::getFF, this::setFF);
    }

    /* ---------- Utilities ---------- */

    /**
     * Update a SparkMaxPIDController with the values in this object. Useful for
     * tuning from the dashboard.
     * <p>
     * <b>IMPORTANT:</b> These settings will not persist unless the
     * {@link com.revrobotics.CANSparkMax#burnFlash() CANSparkMax.burnFlash()}
     * method is called.
     * 
     * @param toUpdate {@link SparkMaxPIDController} to update
     */
    public void updateSparkMax(SparkMaxPIDController toUpdate) {
        // Method is synchronized to prevent simultaneous updates to the same PIDF
        // object
        synchronized (toUpdate) {
            if (toUpdate.getP() != kP)
                toUpdate.setP(kP);
            if (toUpdate.getI() != kI)
                toUpdate.setI(kI);
            if (toUpdate.getD() != kD)
                toUpdate.setD(kD);
            if (toUpdate.getIZone() != kIZone)
                toUpdate.setIZone(kIZone);
            if (toUpdate.getFF() != kFF)
                toUpdate.setFF(kFF);
        }
    }

    /**
     * Update a WPILib PID Controller
     * 
     * @param toUpdate the PID Controller object to update
     */
    public void updatePID(PIDController toUpdate) {
        // Synchronized to prevent updates to the same controller object
        synchronized (toUpdate) {
            synchronized (toUpdate) {
                if (toUpdate.getP() != kP)
                    toUpdate.setP(kP);
                if (toUpdate.getI() != kI)
                    toUpdate.setI(kI);
                if (toUpdate.getD() != kD)
                    toUpdate.setD(kD);
            }
        }
    }

    /**
     * Update a WPILib Profiled PID Controller
     * 
     * @param toUpdate the Profiled PID Controller object to update
     */
    public void updatePID(ProfiledPIDController toUpdate) {
        // Synchronized to prevent updates to the same controller object
        synchronized (toUpdate) {
            synchronized (toUpdate) {
                if (toUpdate.getP() != kP)
                    toUpdate.setP(kP);
                if (toUpdate.getI() != kI)
                    toUpdate.setI(kI);
                if (toUpdate.getD() != kD)
                    toUpdate.setD(kD);
            }
        }
    }

    /* ---------- Getters and Setters ---------- */
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
