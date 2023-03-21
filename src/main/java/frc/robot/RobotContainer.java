// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.Arm.ArmDefaultControlCommand;
import frc.robot.commands.Arm.Telescope.ArmInCommand;
import frc.robot.commands.Arm.Telescope.ArmOutCommand;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.SlowedWhileActiveCommand;
import frc.robot.commands.Drive.SwitchSpeed;
import frc.robot.commands.Intake.CloseIntakeCommand;
import frc.robot.commands.Intake.OpenIntakeCommand;
import frc.robot.commands.Intake.RunIntakeWheelsCommand;
import frc.robot.commands.Intake.ToggleIntakeCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.PneumaticFeetSubsystem;

/**
 * Very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static RobotContainer getInstance() {
        if (INSTANCE == null)
            INSTANCE = new RobotContainer();
        return INSTANCE;
    }

    // The robot's subsystems and commands are defined here...
    DiffDriveSubsystem driveSubsystem = DiffDriveSubsystem.getInstance();
    PigeonSubsystem pigeonSubsystem = PigeonSubsystem.getInstance();
    ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    LimelightSubsytem limelightSubsytem = LimelightSubsytem.getInstance();

    public final RobotState robotState = RobotState.getInstance();
    private AutoPaths autoPaths = AutoPaths.getInstance();

    private static DriverController driver;
    private static OperatorController operator;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        initControllers();

        // Configure the button bindings
        configureButtonBindings();

        // Default Drive Command
        driveSubsystem.setDefaultCommand(new ArcadeDrive(driver));

        // Default Arm Control Command
        armSubsystem.setDefaultCommand(new ArmDefaultControlCommand());

        // Stops the DS from constantly yelling about the joystick being disconnected
        DriverStation.silenceJoystickConnectionWarning(true);

        if (DriverStation.isFMSAttached() == false) {
            PathPlannerServer.startServer(9015);
        }
    }

    public void initRobot() {
        pigeonSubsystem.resetAngles();
        Dashboard.getInstance().putSendable("RobotState", robotState);

        initControllers();

        autoPaths.init();

        Dashboard.getInstance().addAutoPathChooser(autoPaths.getChooser());
    }

    private void initControllers() {
        if (driver == null)
            driver = new DriverController(new XboxController(0));
        Dashboard.getInstance().putSendable("Driver", driver);
        if (operator == null)
            operator = new OperatorController(new XboxController(1));
        Dashboard.getInstance().putSendable("Operator", operator);
    }

    /**
     * Assign Buttons to Command Triggers
     */
    private void configureButtonBindings() {
        initControllers();
        /*
         * driver.getUpDpad().whileTrue(new
         * WeightForwardCommand(counterweightPIDSubsystem));
         * driver.getDownDpad().whileTrue(new
         * WeightBackCommand(counterweightPIDSubsystem));
         */

        /* -------------------- DRIVER CONTROLS -------------------- */

        /*
         * While the driver's Start button is pressed, balance the robot
         * The balance command does have an end condition, but it gets continuously
         * rescheduled
         * while the toggle is active.
         */
        driver.getStart().toggleOnTrue(new RepeatCommand(new BalanceCommand()));

        /* Toggle the feet up and down with the driver's Back button */
        driver.getBack().onTrue(new InstantCommand(() -> PneumaticFeetSubsystem.getInstance().toggleFeet(),
                PneumaticFeetSubsystem.getInstance()));

        /* While the driver's left bumper is held, the robot drives slowly */
        driver.getLb().onTrue(new SlowedWhileActiveCommand());

        /* -------------------- OPERATOR CONTROLS -------------------- */
        /* Open intake while active, remain closed while not pressed */
        operator.getRTriggerAsButton().whileTrue(new OpenIntakeCommand()).whileFalse(new CloseIntakeCommand());

        /* Run intake OUT */
        operator.getLeftBumper().whileTrue(new RunIntakeWheelsCommand(-0.5));
        
        /* Run intake IN */
        operator.getLTriggerAsButton().whileTrue(new RunIntakeWheelsCommand(0.8));
    }

    public DriverController getDriver() {
        return driver;
    }

    public OperatorController getOperator() {
        return operator;
    }

    /**
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Read the selected trajectory from the Dashboard and transform that into a
        // Ramsete command
        return driveSubsystem.getTrajectoryCommand(autoPaths.getSelectedTrajectory(), true);
    }

    /**
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return null; // null as this is already handled by the drive subsystem's default command
    }
}
