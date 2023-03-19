// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.*;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
//import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CounterweightPIDSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FootSubsystem;
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
    private static boolean hasInstantiate = false;
    private static RobotContainer INSTANCE = new RobotContainer();

    public static RobotContainer getInstance() {
        if (INSTANCE == null && !hasInstantiate) {
            System.out.println("---------- CREATING NEW ROBOT-CONTAINER ----------");
            INSTANCE = new RobotContainer();
            hasInstantiate = true;
        }
        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    DiffDriveSubsystem driveSubsystem = DiffDriveSubsystem.getInstance();
    PigeonSubsystem pigeonSubsystem = PigeonSubsystem.getInstance();
    ArmSubsystem armSubsystem= ArmSubsystem.getInstance();
    LimelightSubsytem limelightSubsytem = LimelightSubsytem.getInstance();
    FootSubsystem footSubsystem = FootSubsystem.getInstance();
    //CounterweightSubsystem counterweightSubsystem = CounterweightSubsystem.getInstance();
    CounterweightPIDSubsystem counterweightPIDSubsystem = CounterweightPIDSubsystem.getInstance();
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

        PathPlannerServer.startServer(9015);

        init();

        driveSubsystem.setDefaultCommand(new ArcadeDrive(driver));
        armSubsystem.setDefaultCommand(new armDefualtControlCommand(armSubsystem,operator));
    }

    public void initRobot() {
        pigeonSubsystem.resetAngles();
        Dashboard.getInstance().putSendable("RobotState", robotState);

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

    public Command getAutonomousCommand()
    {

        return(new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0.4, 0),driveSubsystem))).withTimeout(2).andThen(new InstantCommand(()->driveSubsystem.arcadeDrive(0, 0)).andThen(new BalanceCommand(pigeonSubsystem, driveSubsystem)));
    }

    public Command getAutonomousCommandcopy()
    {

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Balance", new BalanceCommand(pigeonSubsystem, driveSubsystem));
        eventMap.put("marker2", new PrintCommand("marker 2"));
        return driveSubsystem.getAutCommandWithEvents(autoPaths.getSelectedTrajectory(), true,eventMap);
    }
    /**
     * Assign Buttons to Command Triggers
     */


     private void configureButtonBindingscopy()
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
         
         driver.getA().toggleOnTrue(new BalanceCommand(pigeonSubsystem, driveSubsystem));
 
         // When the driver's left bumper is pressed, switch between low and high speed.
         //driver.getLB().whileTrue(new StartEndCommand(() -> armSubsystem.SetActivatePID(true),() -> armSubsystem.SetActivatePID(true), armSubsystem));
         driver.getBack().whileTrue(new armpidCommand(armSubsystem, -100,0));
         
         driver.getLB().onTrue(new InstantCommand(()-> intakeNewmaticSubsystem.switchIntake(), intakeNewmaticSubsystem));
 
         driver.getB().onTrue(new WeightCalibrationCommand(counterweightPIDSubsystem));
 
         driver.getUpDpad().whileTrue(new ArmUp(armSubsystem));
         driver.getDownDpad().whileTrue(new ArmDown(armSubsystem));
 
         driver.getY().whileTrue(new ArmInCommand(armSubsystem));
         driver.getX().whileTrue(new ArmOutCommand(armSubsystem));
 
         driver.getLeftDpad().whileTrue(new WeightBackCommand(counterweightPIDSubsystem));
         driver.getRightDpad().whileTrue(new WeightForwardCommand(counterweightPIDSubsystem));
 
         driver.getRTrigAsButton().whileTrue(new StartEndCommand(
             () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.8), 
             ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
             intakeNewmaticSubsystem));
             
         driver.getRB().whileTrue(new StartEndCommand(
             () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.8), 
             ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
             intakeNewmaticSubsystem));
         //driver.getX().onTrue(new OpenIntakeCommand(intakeNewmaticSubsystem));
         driver.getY().whileTrue(new SyncLimelightPose(limelightSubsytem, driveSubsystem));
         driver.getX().onTrue(new InstantCommand(()->driveSubsystem.runRamseteCommand(new Pose2d(1.296, 1.041, new Rotation2d(0.0265)),driveSubsystem)));
 
         //driver.getLB().whileTrue(new PointToTagCommand(limelightSubsytem, driveSubsystem));
         //driver.getRB().whileTrue(new SyncLimelightPose(limelightSubsytem, driveSubsystem));
         
         driver.getStart().onTrue(new waypointCommand(limelightSubsytem, driveSubsystem));
         //RamseteCommand drivRamseteCommand = driveSubsystem.getRamseteCommand(driveSubsystem.getPose(), RobotState.getSavedPoint(), driveSubsystem);
         //driver.getBack().onTrue(new RavisRamseteCommand(driveSubsystem));
 
     }



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
        //driver.getA().toggleOnTrue(new BalanceCommand(pigeonSubsystem, driveSubsystem));
        //driver.getB().onTrue(new WeightCalibrationCommand(counterweightPIDSubsystem));
        //driver.getY().whileTrue(new ArmInCommand(armSubsystem));
        //driver.getX().whileTrue(new ArmOutCommand(armSubsystem));
        
        driver.getUpDpad().whileTrue(new WeightForwardCommand(counterweightPIDSubsystem));
        driver.getDownDpad().whileTrue(new WeightBackCommand(counterweightPIDSubsystem));
        //driver.getLeftDpad().whileTrue(new WeightBackCommand(counterweightPIDSubsystem));
        //driver.getRightDpad().whileTrue(new WeightForwardCommand(counterweightPIDSubsystem));

        // When the driver's left bumper is pressed, switch between low and high speed.
        //driver.getLB().whileTrue(new StartEndCommand(() -> armSubsystem.SetActivatePID(true),() -> armSubsystem.SetActivatePID(true), armSubsystem));
        driver.getStart().toggleOnTrue(new BalanceCommand(pigeonSubsystem, driveSubsystem));
        driver.getBack().onTrue(new InstantCommand(()->footSubsystem.toggleFoot() ,footSubsystem));
        
        operator.getRTrigAsButton().whileTrue(new InstantCommand(()-> intakeNewmaticSubsystem.openIntake(), intakeNewmaticSubsystem)).whileFalse(new InstantCommand(()-> intakeNewmaticSubsystem.closeIntake(), intakeNewmaticSubsystem));
        driver.getRB().whileTrue(new StartEndCommand(
            () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.5), 
            ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
            intakeNewmaticSubsystem));

        driver.getRTrigAsButton().whileTrue(new StartEndCommand(
            () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.8), 
            ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
            intakeNewmaticSubsystem));
            
        driver.getLTrigAsButton().onTrue(new SwitchSpeed());

        //driver.getX().onTrue(new OpenIntakeCommand(intakeNewmaticSubsystem));
      
        //driver.getLB().whileTrue(new PointToTagCommand(limelightSubsytem, driveSubsystem));
        //driver.getRB().whileTrue(new SyncLimelightPose(limelightSubsytem, driveSubsystem));
        
        //RamseteCommand drivRamseteCommand = driveSubsystem.getRamseteCommand(driveSubsystem.getPose(), RobotState.getSavedPoint(), driveSubsystem);
        //driver.getBack().onTrue(new RavisRamseteCommand(driveSubsystem));



        operator.getA().onTrue(new armpidCommand(armSubsystem, 100,0));
        operator.getB().onTrue(new armpidCommand(armSubsystem, 100,0));
        operator.getX().onTrue(new armpidCommand(armSubsystem, 100,0));
        operator.getY().onTrue(new armpidCommand(armSubsystem, 100,0));




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


    /**
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return null; // null as this is already handled by the drive subsystem's default command
    }
}
