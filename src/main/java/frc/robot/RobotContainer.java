// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmInCommand;
import frc.robot.commands.ArmOutCommand;
import frc.robot.commands.ArmUp;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.CloseIntakeCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SyncLimelightPose;
import frc.robot.commands.OpenIntakeCommand;
import frc.robot.commands.PointToTagCommand;
import frc.robot.commands.RavisRamseteCommand;
import frc.robot.commands.SwitchSpeed;
import frc.robot.commands.WeightBackCommand;
import frc.robot.commands.WeightCalibrationCommand;
import frc.robot.commands.WeightForwardCommand;
import frc.robot.commands.waypointCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
//import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightPIDSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeNewmaticSubsystem;
import frc.robot.subsystems.LimelightSubsytem;
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
    ArmSubsystem armSubsystem= ArmSubsystem.getInstance();
    LimelightSubsytem limelightSubsytem = LimelightSubsytem.getInstance();
    //CounterweightSubsystem counterweightSubsystem = CounterweightSubsystem.getInstance();
    CounterweightPIDSubsystem counterweightPIDSubsystem = CounterweightPIDSubsystem.getInstance();
    private final Command autoCommand = new ExampleCommand(exampleSubsystem);
    IntakeNewmaticSubsystem intakeNewmaticSubsystem = IntakeNewmaticSubsystem.getInstance();
    // private final Command driveCommand = new ArcadeDrive();

    public final RobotState robotState = RobotState.getInstance();
    private AutoPaths autoPaths = AutoPaths.getInstance();

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
        //Dashboard.getInstance().putData("RobotState",robotState);
        init();

        autoPaths.init();

        Dashboard.getInstance().addAutoPathChooser(autoPaths.getChooser());
    }

    private void init() {
        if (driver == null) driver = new DriverController(new XboxController(0));
        Dashboard.getInstance().putSendable("Driver", driver);
        if (operator == null) operator = new OperatorController(new XboxController(1));
        Dashboard.getInstance().putSendable("Operator", operator);
    }
    
    
    /**
     * Assign Buttons to Command Triggers
     */
    private void configureButtonBindings()
    {
        init();
        /*button bindings:
        ABXY buttons for setting the arm/intake to preset locations for scoring and intaking
        dpad: up/down for adjusting the arm, left/right for slowly rotating the robot for the sake of scoring
        left trigger/bumper: deployabe intake rollers
        right trigger/bumper: intake claw
        start/select: balancing/foot
        6

        */
        // Toggle the balance command on and off when the driver's A button is pressed
        driver.getA().toggleOnTrue(new RepeatCommand(new BalanceCommand(pigeonSubsystem, driveSubsystem)));

        // When the driver's left bumper is pressed, switch between low and high speed.
        driver.getLB().onTrue(new SwitchSpeed());


        driver.getB().onTrue(new WeightCalibrationCommand(counterweightPIDSubsystem));

        driver.getUpDpad().whileTrue(new ArmUp(armSubsystem));
        driver.getDownDpad().whileTrue(new ArmDown(armSubsystem));

        driver.getY().whileTrue(new ArmInCommand(armSubsystem));
        driver.getX().whileTrue(new ArmOutCommand(armSubsystem));

        driver.getLeftDpad().whileTrue(new WeightBackCommand(counterweightPIDSubsystem));
        driver.getRightDpad().whileTrue(new WeightForwardCommand(counterweightPIDSubsystem));

        driver.getX().onTrue(new OpenIntakeCommand(intakeNewmaticSubsystem));
        driver.getY().onTrue(new CloseIntakeCommand(intakeNewmaticSubsystem));

        driver.getLB().whileTrue(new PointToTagCommand(limelightSubsytem, driveSubsystem));
        driver.getRB().whileTrue(new SyncLimelightPose(limelightSubsytem, driveSubsystem));
        
        driver.getStart().onTrue(new waypointCommand(limelightSubsytem, driveSubsystem));
        //RamseteCommand drivRamseteCommand = driveSubsystem.getRamseteCommand(driveSubsystem.getPose(), RobotState.getSavedPoint(), driveSubsystem);
        driver.getBack().onTrue(new RavisRamseteCommand(driveSubsystem));

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
        // Read the selected trajectory from the Dashboard and transform that into a Ramsete command
        return driveSubsystem.getTrajectoryCommand(autoPaths.getSelectedTrajectory(), true);
    }

    /**
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return null; // null as this is already handled by the drive subsystem's default command
    }
}
