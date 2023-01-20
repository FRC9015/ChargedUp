// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwitchSpeed;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;

/**
 * Very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer INSTANCE = new RobotContainer();

    @SuppressWarnings("WeakerAccess")
    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            System.out.println("---------- CREATING NEW ROBOT-CONTAINER ----------");
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    DiffDriveSubsystem driveSubsystem = DiffDriveSubsystem.getInstance();
    PigeonSubsystem pigeonSubsystem = PigeonSubsystem.getInstance();
    CounterweightSubsystem counterweightSubsystem = CounterweightSubsystem.getInstance();

    private final Command autoCommand = new ExampleCommand(exampleSubsystem);
    // private final Command driveCommand = new ArcadeDrive();

    public final RobotState robotState = RobotState.getInstance();

    private DriverController driver;
    private OperatorController operator;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();

        init();

        driveSubsystem.setDefaultCommand(new ArcadeDrive(driver));
    }

    public void initRobot() {
        pigeonSubsystem.resetAngles();
        Dashboard.getInstance().putData("RobotState",robotState);
        init();
    }

    private void init() {
        if (driver == null) driver = new DriverController(new PS4Controller(0));
        Dashboard.getInstance().putData("Driver", driver);
        if (operator == null) operator = new OperatorController(new XboxController(1));
        Dashboard.getInstance().putData("Operator", operator);
    }
    
    
    /**
     * Assign Buttons to Command Triggers
     */
    private void configureButtonBindings()
    {
        init();

        // Toggle the balance command on and off when the driver's A button is pressed
        driver.getBalanceButton().toggleOnTrue(new RepeatCommand(new BalanceCommand(pigeonSubsystem, driveSubsystem)));

        // When the driver's left bumper is pressed, switch between low and high speed.
        driver.getSwitchSpeed().onTrue(new SwitchSpeed());

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
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }

    /**
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return null; // null as this is already handled by the drive subsystem's default command
    }
}
