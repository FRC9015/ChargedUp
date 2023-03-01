package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import java.util.List;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Dashboard;
import frc.robot.RobotState;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.UpdatePIDFConstantsCommand;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.utils.PIDFConstants;

public class DiffDriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are
    // initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DiffDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DiffDriveSubsystem INSTANCE = new DiffDriveSubsystem();

    /**
     * Returns the Singleton instance of this DiffDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DiffDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DiffDriveSubsystem getInstance() {
        return INSTANCE;
    }

    private final CANSparkMax left1, left2;
    private final CANSparkMax right1, right2;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final SparkMaxPIDController leftPID, rightPID;
    private final RamseteController trajRamsete;
    private final DifferentialDriveOdometry odometry;
    private final PIDFConstants velocityPIDFConstants;

    // private final Dashboard dash = Dashboard.getInstance();

    private final Field2d field;

    /**
     * BiConsumer function that accepts a left and right double values for meters
     * per second
     */
    private BiConsumer<Double, Double> ramseteOutputBiConsumer;

    private IdleMode brakeMode = IdleMode.kCoast;

    private final SlewRateLimiter accelRateLimit1, accelRateLimit2;

    private ArrayList<CANSparkMax> allMotors = new ArrayList<CANSparkMax>();

    private PigeonSubsystem pigeon = PigeonSubsystem.getInstance();

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    /**
     * Creates a new instance of this DiffDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DiffDriveSubsystem() {

        CANSparkMax.enableExternalUSBControl(true);

        MotorType motorType = MotorType.kBrushless;
        left1 = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_ID, motorType);
        allMotors.add(left1);
        left2 = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_ID, motorType);
        allMotors.add(left2);

        right1 = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_ID, motorType);
        allMotors.add(right1);
        right2 = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_ID, motorType);
        allMotors.add(right2);

        left2.follow(left1);

        right2.follow(right1);

        // Properly invert motors
        left1.setInverted(true);
        right1.setInverted(false);

        left1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        right2.setIdleMode(CANSparkMax.IdleMode.kBrake);


        leftEncoder = left1.getEncoder();
        leftEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        leftEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        rightEncoder = right1.getEncoder();
        rightEncoder.setPositionConversionFactor(DriveConstants.DRIVE_ENCODER_POSITION_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        leftPID = left1.getPIDController();

        rightPID = right1.getPIDController();

        // Create a new PIDFConstants object for the drive
        velocityPIDFConstants = new PIDFConstants(0.05, 0, 0, 0, 0.000007);

        double kMaxOutput = 1;
        double kMinOutput = -1;
        
        velocityPIDFConstants.updateSparkMax(leftPID);
        leftPID.setOutputRange(kMinOutput, kMaxOutput);


        velocityPIDFConstants.updateSparkMax(rightPID);
        rightPID.setOutputRange(kMinOutput, kMaxOutput);

        Dashboard.getInstance().putSendable("Drive Velocity PIDF/Constants", velocityPIDFConstants);
        Dashboard.getInstance().putSendable("Drive Velocity PIDF/Update",
                new UpdatePIDFConstantsCommand(velocityPIDFConstants, leftPID, rightPID));

        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(),
                leftEncoder.getPosition(), rightEncoder.getPosition());
        field = new Field2d();
        //addChild("Field", field);
        //Dashboard.getInstance().putSendable("field", field);
        SmartDashboard.putData("field",field);
        /**
         * Each input to be rate limited must have it's own filter. In any given drive,
         * we have two possible inputs, and thus two filters.
         * 1: used for the Left input (Tank) and the Forward input (Arcade)
         * 2: used for the Right input (Tank) and the Turn input (Arcade)
         */
        accelRateLimit1 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);
        accelRateLimit2 = new SlewRateLimiter(DriveConstants.ACCEL_RATE_LIMIT);

        trajRamsete = new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA);

        ramseteOutputBiConsumer = (left, right) -> {
            setSpeeds(new DifferentialDriveWheelSpeeds(left, right));
        };
    }

    @Override
    public void periodic() {
        odometry.update(pigeon.getRotation2d(), leftEncoder.getPosition(),
                rightEncoder.getPosition());
        field.setRobotPose(odometry.getPoseMeters());

    }

    public void arcadeDrive(double fwd, double turn) {
        arcadeDriveRaw(fwd, turn, true);
    }

    public void tankDrive(double left, double right) {
        tankDriveRaw(left, right, true);
    }

    /**
     * 
     * @param fwd         Forward Speed
     * @param turn        Turning Speed
     * @param rateLimited Should the inputs be rate limited?
     */
    public void arcadeDriveRaw(double fwd, double turn, boolean rateLimited) {
        fwd = rateLimited ? accelRateLimit1.calculate(fwd) : fwd;
        //turn = rateLimited ? accelRateLimit2.calculate(turn) : turn;

        //double fwdSpeed = calcMetersPerSecond(fwd);
        //double turnSpeed = calcRadiansPerSecond(turn);


        setSpeedsRaw(fwd-turn,fwd+turn);
    }

    /**
     * Tank Drive the robot without any speed multiplier and optional rate limiting
     * 
     * @param left        Left Speed
     * @param right       Right Speed
     * @param rateLimited Should the input be rate limited?
     */
    public void tankDriveRaw(double left, double right, boolean rateLimited) {
        left = rateLimited ? accelRateLimit1.calculate(left) : left;
        right = rateLimited ? accelRateLimit2.calculate(right) : right;

        double leftSpeed = calcMetersPerSecond(left);
        double rightSpeed = calcMetersPerSecond(right);

        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);

        setSpeeds(wheelSpeeds);
    }
    //issue: the left is just folowing the right

    private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        
        leftPID.setReference(speeds.leftMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        rightPID.setReference(speeds.rightMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }

    private void setSpeedsRaw(double left, double right) {
        
        left1.set(left);
        right1.set(right);

    }

    public synchronized void setBrakeMode(IdleMode newBrakeMode) {
        this.brakeMode = newBrakeMode;
        for (CANSparkMax controller : allMotors) {
            controller.setIdleMode(newBrakeMode);
        }
    }

    public void stop() {
        setSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }

    public void brakeStop() {
        IdleMode prevBrakeMode = this.brakeMode;
        setBrakeMode(IdleMode.kBrake);
        stop();
        setBrakeMode(prevBrakeMode);
    }
    
    public void resetOdometry(Pose2d initPose) {
        resetEncoders();
        odometry.resetPosition(pigeon.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initPose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }




    public void runRamseteCommand(Pose2d start, Pose2d end,DiffDriveSubsystem m_diffDriveSubsystem) {
        
    
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(1, 0.5);
        Translation2d idk = new Translation2d();
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(idk);
        // An example trajectory to follow.  All units in meters.
        Trajectory trajectorytogo =
            TrajectoryGenerator.generateTrajectory(
                start,
                List.of(),
                end,
                config);
    
        RamseteCommand ramseteCommand =
            new RamseteCommand(trajectorytogo,odometry::getPoseMeters,new RamseteController(),DriveConstants.KINEMATICS,ramseteOutputBiConsumer,m_diffDriveSubsystem);
    
        System.out.println("ramseteCommand");
    
        ramseteCommand.schedule();
      }
    
    public Command getTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        odometry.resetPosition(pigeon.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
        return new SequentialCommandGroup(new InstantCommand(() -> {
            if (isFirstPath) {
                resetOdometry(traj.getInitialPose());
            }
        }),
                /*
                 * Note: currently does not perform transforms based on Alliance Color
                 *
                 * We do not supply PIDControllers or Feedforwards, instead we take the
                 * velocity output from the RamseteController which is then processed by the
                 * SparkMax's built in PID and Feedforward
                 * functionality
                 */
                new PPRamseteCommand(traj, this::getPose, trajRamsete, DriveConstants.KINEMATICS,
                        ramseteOutputBiConsumer,
                        this));
    }
    
    /**
     * Takes in a joystick input and converts it to meters per second, taking into
     * account slow mode
     * 
     * @param input Joystick input, within range [-1, 1]
     * @return meters per second
     */
    private double calcMetersPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        double inputMetersPerSecond = (input * DriveConstants.MAX_RPM) * DriveConstants.DRIVE_ENCODER_VELOCITY_FACTOR;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        // If the robot should be running in slow mode, reduce speed by the multiplier
        // (set in dashboard)
        return isSlowed ? inputMetersPerSecond * speedMultiplier : inputMetersPerSecond;
    }

    public void simulationPeriodic(){
    }

    private double calcRadiansPerSecond(double input) {
        boolean isSlowed = RobotState.getSlowedSmart();

        double inputRadiansPerSecond = input * DriveConstants.MAX_ANGULAR_VELOCITY;

        double speedMultiplier = Dashboard.getInstance().drive.getSpeedMultiplier();

        return isSlowed ? inputRadiansPerSecond * speedMultiplier : inputRadiansPerSecond;
    }

    private void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

}
