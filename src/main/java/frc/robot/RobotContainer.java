// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwitchSpeed;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;


/**
 * Very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final static RobotContainer INSTANCE = new RobotContainer();

    @SuppressWarnings("WeakerAccess")
    public static RobotContainer getInstance() {
        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    DiffDriveSubsystem driveSubsystem = DiffDriveSubsystem.getInstance();
    PigeonSubsystem pigeonSubsystem = PigeonSubsystem.getInstance();

    private final Command autoCommand = new ExampleCommand(exampleSubsystem);
    private final Command driveCommand = new ArcadeDrive();

    public final RobotState robotState = RobotState.getInstance();
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();

        driveSubsystem.setDefaultCommand(driveCommand);
    }

    public void initRobot() {
        pigeonSubsystem.resetAngles();
    }
    
    
    /**
     * Assign Buttons to Command Triggers
     */
    private void configureButtonBindings()
    {
        JoystickButton driveAButton = new JoystickButton(getDriverJoystick(), XboxController.Button.kA.value);
        // Toggle the balance command on and off when the driver's A button is pressed
        driveAButton.toggleOnTrue(new BalanceCommand(pigeonSubsystem, driveSubsystem));

        JoystickButton driveLBumper = new JoystickButton(getDriverJoystick(), XboxController.Button.kLeftBumper.value);
        // When the driver's left bumper is pressed, switch between low and high speed.
        driveLBumper.onTrue(new SwitchSpeed());

    }

    public XboxController getDriverJoystick() {
        return new XboxController(0);
    }

    public XboxController getOperatorJoystick() {
        return new XboxController(1);
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
