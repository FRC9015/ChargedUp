package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDPreset;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private Command teleopCommand;

    private DigitalInput calibrationLimitSwitch = new DigitalInput(3);
    private DigitalInput brakeToggle = new DigitalInput(4);

    private RobotContainer robotContainer;
    private AutoPaths autoPaths;

    private LEDSubsystem leds;
    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer.getInstance();

        robotContainer.initRobot();

        autoPaths = AutoPaths.getInstance();

        // CameraServer.startAutomaticCapture();

        leds = LEDSubsystem.getInstance();
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        Dashboard.getInstance().periodic();
    }

    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        if (calibrationLimitSwitch.get()) {
            ArmSubsystem.getInstance().resetArm();
            // if(DriverStation.isFMSAttached()){
            leds.setPreset(LEDPreset.GREEN);

            // }else{
            // robotContainer.getLedSubsystem().setPreset(LEDPreset.OFF);
            // }
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        DiffDriveSubsystem.getInstance().resetOdometry(new Pose2d());
        // DiffDriveSubsystem.getInstance().runRamseteCommand(new Pose2d(0, 0, new Rotation2d()),
        // new Pose2d(0, 1, new Rotation2d()), DiffDriveSubsystem.getInstance());
        // Dashboard.getInstance().setCurrentTab(CurrentTab.Auto);
        autonomousCommand = autoPaths.getSelectedAuto();

        // schedule the autonomous command
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Dashboard.getInstance().setCurrentTab(CurrentTab.TeleOp);
        // Dashboard.getInstance().setCurrentTab(CurrentTab.TeleOp);

        // On teleop init, make sure that the dashboard does not continue to show the robot as
        // AutoBalanced
        Dashboard.getInstance().balance.setAutoBalanced(false);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        if (teleopCommand != null) {
            teleopCommand.schedule();
        }

        FootSubsystem.getInstance().footUp();
    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Update the dashboard to show whether the robot is slowed
        Dashboard.getInstance().drive.setSpeedMode(RobotState.getSlowed());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        // CommandScheduler.getInstance().cancelAll();
    }

    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        Dashboard.getInstance().balance.setAutoBalanced(true);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        if (teleopCommand != null) {
            teleopCommand.schedule();
        }
    }
}
