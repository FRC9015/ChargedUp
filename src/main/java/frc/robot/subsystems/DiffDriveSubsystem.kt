package frc.robot.subsystems

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.PPRamseteCommand
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.IdleMode
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import frc.robot.Dashboard
import frc.robot.Helpers
import frc.robot.RobotState
import frc.robot.commands.UpdatePIDFConstantsCommand
import frc.robot.utils.PIDFConstants
import java.util.function.BiConsumer
import java.util.ArrayList

object DiffDriveSubsystem : SubsystemBase() {
    private val leftFront: CANSparkMax
    private val leftBack: CANSparkMax
    private val rightFront: CANSparkMax
    private val rightBack: CANSparkMax

    private var brakeMode = IdleMode.kCoast
    private val allMotors = ArrayList<CANSparkMax>()

    private val leftEncoder: RelativeEncoder
    private val rightEncoder: RelativeEncoder

    private val leftPID: SparkMaxPIDController
    private val rightPID: SparkMaxPIDController

    private val trajRamsete: RamseteController
    private val odometry: DifferentialDriveOdometry
    private val field: Field2d

    private val velocityPIDFConstants: PIDFConstants

    private val dash = Dashboard.getInstance()
    private val leftSpeed = dash.putNumber("Speed/Left", 0.0)
    private val leftSpeedActual = dash.putNumber("Speed/LeftActual", 0.0)
    private val rightSpeed = dash.putNumber("Speed/Right", 0.0)
    private val rightSpeedActual = dash.putNumber("Speed/RightActual", 0.0)

    /** BiConsumer function that accepts a left and right double values for meters per second */
    private val ramseteOutputBiConsumer: BiConsumer<Double, Double>

    private val accelRateLimit1: SlewRateLimiter
    private val accelRateLimit2: SlewRateLimiter

    private val pigeon = PigeonSubsystem.getInstance()

    /**
     */
    init {
        CANSparkMax.enableExternalUSBControl(false)
        val motorType = CANSparkMaxLowLevel.MotorType.kBrushless
        leftFront = CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, motorType)
        allMotors.add(leftFront)
        leftBack = CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, motorType)
        allMotors.add(leftFront)
        rightFront = CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType)
        allMotors.add(rightFront)
        rightBack = CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType)
        allMotors.add(rightBack)
        leftBack.follow(leftFront)
        rightBack.follow(rightFront)

        // Properly invert motors
        leftFront.inverted = DriveConstants.LEFT_INVERTED
        rightFront.inverted = DriveConstants.RIGHT_INVERTED
        leftEncoder = leftFront.encoder
        leftEncoder.positionConversionFactor = DriveConstants.DRIVE_ENCODER_POSITION_FACTOR
        leftEncoder.velocityConversionFactor = DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR
        rightEncoder = rightFront.encoder
        rightEncoder.positionConversionFactor = DriveConstants.DRIVE_ENCODER_POSITION_FACTOR
        rightEncoder.velocityConversionFactor = DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR
        leftPID = leftFront.pidController
        rightPID = rightFront.pidController

        // Create a new PIDFConstants object for the drive
        velocityPIDFConstants = PIDFConstants(0.05, 0.0, 0.0, 0.0, 0.099)
        for (ctrl in allMotors) {
            ctrl.clearFaults()
            println(ctrl.stickyFaults)
            ctrl.burnFlash()
            ctrl.idleMode = brakeMode
        }
        val kMaxOutput = 1.0
        val kMinOutput = -1.0
        velocityPIDFConstants.updateSparkMax(leftPID)
        leftPID.setOutputRange(kMinOutput, kMaxOutput)
        velocityPIDFConstants.updateSparkMax(rightPID)
        rightPID.setOutputRange(kMinOutput, kMaxOutput)
        Dashboard.getInstance().putSendable("Drive Velocity PIDF/Constants", velocityPIDFConstants)
        Dashboard.getInstance()
                .putSendable(
                        "Drive Velocity PIDF/Update",
                        UpdatePIDFConstantsCommand(velocityPIDFConstants, leftPID, rightPID)
                )
        odometry =
                DifferentialDriveOdometry(
                        pigeon.rotation2d,
                        leftEncoder.position,
                        rightEncoder.position
                )
        field = Field2d()
        // addChild("Field", field);
        // Dashboard.getInstance().putSendable("field", field);
        SmartDashboard.putData("field", field)
        /**
         * Each input to be rate limited must have it's own filter. In any given drive, we have two
         * possible inputs, and thus two filters. 1: used for the Left input (Tank) and the Forward
         * input (Arcade) 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT_1)
        accelRateLimit2 = SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT_2)
        trajRamsete = RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA)
        ramseteOutputBiConsumer = BiConsumer { left: Double?, right: Double? ->
            setSpeeds(DifferentialDriveWheelSpeeds(left!!, right!!))
        }
    }

    override fun periodic() {
        val rot2d = Rotation2d(pigeon.rotation2d.radians - Math.PI)
        odometry.update(rot2d, leftEncoder.position, rightEncoder.position)
        field.robotPose = odometry.poseMeters
        leftSpeedActual.entry.setDouble(leftEncoder.velocity)
        rightSpeedActual.entry.setDouble(rightEncoder.velocity)
    }

    val pIDFUpdateCommand: Command
        get() = UpdatePIDFConstantsCommand(velocityPIDFConstants, leftPID, rightPID)

    fun arcadeDrive(fwd: Double, turn: Double) {
        arcadeDriveRaw(fwd, turn, true)
    }

    fun tankDrive(left: Double, right: Double) {
        tankDriveRaw(left, right, true)
    }

    /**
     *
     * @param fwd Forward Speed
     * @param turn Turning Speed
     * @param rateLimited Should the inputs be rate limited?
     */
    fun arcadeDriveRaw(fwd: Double, turn: Double, rateLimited: Boolean) {
        var fwd = fwd
        var turn = turn
        fwd = if (rateLimited) accelRateLimit1.calculate(fwd) else fwd
        turn = if (rateLimited) accelRateLimit2.calculate(turn) else turn
        val fwdSpeed = calcMetersPerSecond(fwd)
        val turnSpeed = calcRadiansPerSecond(turn)
        val wheelSpeeds =
                DriveConstants.KINEMATICS.toWheelSpeeds(ChassisSpeeds(fwdSpeed, 0.0, turnSpeed))
        setSpeeds(wheelSpeeds)
    }

    /**
     * Tank Drive the robot without any speed multiplier and optional rate limiting
     *
     * @param left Left Speed
     * @param right Right Speed
     * @param rateLimited Should the input be rate limited?
     */
    fun tankDriveRaw(left: Double, right: Double, rateLimited: Boolean) {
        var left = left
        var right = right
        left = if (rateLimited) accelRateLimit1.calculate(left) else left
        right = if (rateLimited) accelRateLimit2.calculate(right) else right
        val leftSpeed = calcMetersPerSecond(left)
        val rightSpeed = calcMetersPerSecond(right)
        val wheelSpeeds = DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed)
        setSpeeds(wheelSpeeds)
    }

    @Synchronized
    fun setBrakeMode(newBrakeMode: IdleMode) {
        brakeMode = newBrakeMode
        for (controller in allMotors) {
            controller.idleMode = newBrakeMode
        }
    }

    fun stop() {
        setSpeeds(DifferentialDriveWheelSpeeds(0.0, 0.0))
    }

    fun brakeStop() {
        val prevBrakeMode = brakeMode
        setBrakeMode(IdleMode.kBrake)
        stop()
        setBrakeMode(prevBrakeMode)
    }

    fun resetOdometry(initPose: Pose2d?) {
        odometry.resetPosition(
                pigeon.rotation2d,
                leftEncoder.position,
                rightEncoder.position,
                initPose
        )
    }

    fun logPosition(name: String) {
        Helpers.logBox(
                "Note: $name",
                "Left Enc: " + leftEncoder.position,
                "Right Enc: " + rightEncoder.position,
                "Rotation2D: " + pigeon.rotation2d.degrees,
                "Pose2D: " + odometry.poseMeters
        )
    }

    val pose: Pose2d
        get() = odometry.poseMeters
    val wheelSpeeds: DifferentialDriveWheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftEncoder.velocity, rightEncoder.velocity)

    fun getTrajectoryCommand(traj: PathPlannerTrajectory, isFirstPath: Boolean): Command {
        return SequentialCommandGroup(
                InstantCommand({
                    logPosition("BeforePPReset")
                    if (isFirstPath) {
                        resetOdometry(traj.initialPose)
                        logPosition("AfterPPReset")
                    }
                }), /*
                 * Note: currently does not perform transforms based on Alliance Color
                 *
                 * We do not supply PIDControllers or Feedforwards, instead we take the
                 * velocity output from the RamseteController which is then processed by the
                 * SparkMax's built in PID and Feedforward
                 * functionality
                 */
                PPRamseteCommand(
                        traj,
                        { pose },
                        trajRamsete,
                        DriveConstants.KINEMATICS,
                        ramseteOutputBiConsumer,
                        false,
                        this
                )
        )
    }
    // ----------------- PRIVATE HELPER METHODS ----------------- //
    /**
     * Takes in a joystick input and converts it to meters per second, taking into account slow mode
     *
     * @param input Joystick input, within range [-1, 1]
     * @return meters per second
     */
    private fun calcMetersPerSecond(input: Double): Double {
        var input = input
        val isSlowed = RobotState.getSlowedSmart()
        input = MathUtil.applyDeadband(input, 0.1)
        val inputMetersPerSecond =
                input * DriveConstants.MAX_RPM * DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR
        val speedMultiplier = Dashboard.getInstance().drive.speedMultiplier

        // If the robot should be running in slow mode, reduce speed by the multiplier
        // (set in dashboard)
        return if (isSlowed) inputMetersPerSecond * speedMultiplier else inputMetersPerSecond
    }

    private fun calcRadiansPerSecond(input: Double): Double {
        var input = input
        val isSlowed = RobotState.getSlowedSmart()
        input = MathUtil.applyDeadband(input, 0.1)
        val inputRadiansPerSecond = input * DriveConstants.MAX_ANGULAR_VELOCITY
        val speedMultiplier = Dashboard.getInstance().drive.speedMultiplier
        return if (isSlowed) inputRadiansPerSecond * speedMultiplier else inputRadiansPerSecond
    }

    private fun setSpeeds(speeds: DifferentialDriveWheelSpeeds) {
        leftPID.setReference(speeds.leftMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        rightPID.setReference(speeds.rightMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        leftSpeed.entry.setDouble(speeds.leftMetersPerSecond)
        rightSpeed.entry.setDouble(speeds.rightMetersPerSecond)
    }

    private fun resetEncoders() {
        leftEncoder.position = 0.0
        rightEncoder.position = 0.0
    }
}
