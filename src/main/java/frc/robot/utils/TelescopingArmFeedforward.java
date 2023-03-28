package frc.robot.utils;

/**
 * A helper class that computes feedforward outputs for a telescoping arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle, where the force of gravity can increase or decrease based on the position of the arm).
 * <br></br> Based on https://github.com/wpilibsuite/allwpilib/blob/fbf92e919092fe262b1135f9871c8dcdfc6c7a31/wpimath/src/main/java/edu/wpi/first/math/controller/ArmFeedforward.java
 */

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.InterpolatingTreeMap;

public class TelescopingArmFeedforward {
    public final double ks;
    public final double minKg, maxKg;
    private final InterpolatingTreeMap<Double, Double> kgInterpolate;
    public final double kv;
    public final double ka;

    /**
     * Creates a new ArmFeedforward with the specified gains. Units of the gain
     * values will dictate
     * units of the computed feedforward.
     *
     * @param ks    The static gain.
     * @param minKg The gravity gain when the arm is fully retracted
     * @param maxKg The gravity gain when the arm is fully extended
     * @param kv    The velocity gain.
     * @param ka    The acceleration gain.
     */
    public TelescopingArmFeedforward(double ks, double minKg, double maxKg, double kv, double ka) {
        this.ks = ks;
        this.minKg = minKg;
        this.maxKg = maxKg;
        this.kv = kv;
        this.ka = ka;

        this.kgInterpolate = new InterpolatingTreeMap<Double, Double>();
        kgInterpolate.put(1.0, maxKg);
        kgInterpolate.put(0.0, minKg);
    }

    /**
     * Creates a new ArmFeedforward with the specified gains. Acceleration gain is
     * defaulted to zero.
     * Units of the gain values will dictate units of the computed feedforward.
     *
     * @param ks    The static gain.
     * @param minKg The gravity gain when the arm is fully retracted
     * @param maxKg The gravity gain when the arm is fully extended
     * @param kv    The velocity gain.
     */
    public TelescopingArmFeedforward(double ks, double minKg, double maxKg, double kv) {
        this(ks, minKg, maxKg, kv, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param positionRadians       The position (angle) setpoint. This angle should
     *                              be measured from the
     *                              horizontal (i.e. if the provided angle is 0, the
     *                              arm should be parallel with the floor). If
     *                              your encoder does not follow this convention, an
     *                              offset should be added.
     * @param velocityRadPerSec     The velocity setpoint.
     * @param accelRadPerSecSquared The acceleration setpoint.
     * @param telescoped            How far the arm is extended. In range [0, 1]
     * @return The computed feedforward.
     */
    public double calculate(
            double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared, double telescoped) {
        return ks * Math.signum(velocityRadPerSec)
                + getKg(telescoped) * Math.cos(positionRadians)
                + kv * velocityRadPerSec
                + ka * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration
     * is assumed to be
     * zero).
     *
     * @param positionRadians The position (angle) setpoint. This angle should be
     *                        measured from the
     *                        horizontal (i.e. if the provided angle is 0, the arm
     *                        should be parallel with the floor). If
     *                        your encoder does not follow this convention, an
     *                        offset should be added.
     * @param velocity        The velocity setpoint.
     * @param telescoped      How far the arm is extended. In range [0, 1]
     * @return The computed feedforward.
     */
    public double calculate(double positionRadians, double velocity, double telescoped) {
        return calculate(positionRadians, velocity, telescoped);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply, a
     * position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint,
     * and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm. This angle should be measured from
     *                     the horizontal (i.e. if
     *                     the provided angle is 0, the arm should be parallel with
     *                     the floor). If your encoder does
     *                     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @param telescoped   How far the arm is extended. In range [0, 1]
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration, double telescoped) {
        // Assume max velocity is positive
        return (maxVoltage - ks - Math.cos(angle) * getKg(telescoped) - acceleration * ka) / kv;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply, a
     * position, and an
     * acceleration. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the acceleration constraint,
     * and this will give
     * you a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm. This angle should be measured from
     *                     the horizontal (i.e. if
     *                     the provided angle is 0, the arm should be parallel with
     *                     the floor). If your encoder does
     *                     not follow this convention, an offset should be added.
     * @param acceleration The acceleration of the arm.
     * @param telescoped   How far the arm is extended. In range [0, 1]
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration, double telescoped) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - Math.cos(angle) * getKg(telescoped) - acceleration * ka) / kv;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and
     * this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm. This angle should be measured from
     *                   the horizontal (i.e. if
     *                   the provided angle is 0, the arm should be parallel with
     *                   the floor). If your encoder does
     *                   not follow this convention, an offset should be added.
     * @param velocity   The velocity of the arm.
     * @param telescoped How far the arm is extended. In range [0, 1]
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity, double telescoped) {
        return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * getKg(telescoped) - velocity * kv) / ka;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and
     * a velocity. Useful for ensuring that velocity and acceleration constraints
     * for a trapezoidal
     * profile are simultaneously achievable - enter the velocity constraint, and
     * this will give you a
     * simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm. This angle should be measured from
     *                   the horizontal (i.e. if
     *                   the provided angle is 0, the arm should be parallel with
     *                   the floor). If your encoder does
     *                   not follow this convention, an offset should be added.
     * @param velocity   The velocity of the arm.
     * @param telescoped How far the arm is extended. In range [0, 1]
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity, double telescoped) {
        return maxAchievableAcceleration(-maxVoltage, angle, velocity, telescoped);
    }

    /**
     * Get the kG as a linear interpolated value
     * 
     * @param telescoped How far the arm is telescoped in range [0, 1]
     * @return the linear interpolated kG value
     */
    private double getKg(double telescoped) {
        double val = MathUtil.clamp(telescoped, 0, 1);
        return kgInterpolate.get(val);
    }
}
