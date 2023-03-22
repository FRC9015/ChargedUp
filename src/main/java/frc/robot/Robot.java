package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    private Dashboard dashboard;

    /**
     * Short for Is First robotPeriodic run; this is used for running functions
     * after boot
     */
    private boolean isFirstRPRun = true;

    /* ---------- Base Robot Methods ---------- */
    @Override
    public void robotInit() {
        /* WARNING: The robot does not start until this method finishes! */
        robotContainer = RobotContainer.getInstance();

        dashboard = Dashboard.getInstance();

        robotContainer.robotInit();
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        dashboard.periodic();

        if (isFirstRPRun) {
            robotContainer.robotPostBoot();
            isFirstRPRun = false;
        }
    }

    /* -------------------- Disabled Methods -------------------- */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    /* -------------------- Autonomous Mode Methods -------------------- */

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();

    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    /* -------------------- TeleOp Mode Methods -------------------- */

    @Override
    public void teleopInit() {
        Dashboard.getInstance().balance.setAutoBalanced(false);

        if (autonomousCommand != null)
            autonomousCommand.cancel();

    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Update the dashboard to show whether the robot is slowed
        Dashboard.getInstance().drive.setSpeedMode(RobotState.getSlowed());
    }

    @Override
    public void teleopExit() {
    }

    /* -------------------- Test Mode Methods -------------------- */

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    /* -------------------- Test Mode Methods -------------------- */

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
