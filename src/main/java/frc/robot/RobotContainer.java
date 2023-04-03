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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.Arm.ArmDown;
import frc.robot.commands.Arm.ArmInCommand;
import frc.robot.commands.Arm.ArmOutCommand;
import frc.robot.commands.Arm.ArmUp;
import frc.robot.commands.Arm.armDefualtControlCommand;
import frc.robot.commands.Arm.armpidCommand;
import frc.robot.commands.Balance.BalanceCommand;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.SlowedWhileActiveCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
//import frc.robot.subsystems.CounterweightSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.IntakeNewmaticSubsystem;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.PigeonSubsystem;

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
        armSubsystem.resetArm();
        if (driver == null) driver = new DriverController(new XboxController(0));
        Dashboard.getInstance().putSendable("Driver", driver);
        if (operator == null) operator = new OperatorController(new XboxController(1));
        Dashboard.getInstance().putSendable("Operator", operator);

        //armSubsystem.resetArm();
    
    }

    public Command getHighCubeMobilzeBalanceAuto()
    {
        return(new SequentialCommandGroup(
            new PrintCommand("getHighCubeMobilzeBalanceAuto"),
            new armpidCommand(armSubsystem, 0.81,0.6,true,0.1,operator).withTimeout(3).alongWith(
                new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.1), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.25)
            ),
            new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.5), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.2),
            new ParallelCommandGroup(
            new armpidCommand(armSubsystem, 0.60,0.13,true,0.1,operator),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0.3, 0),driveSubsystem)).withTimeout(4.1)),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0, 0),driveSubsystem)).withTimeout(0.8),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(-0.4, 0),driveSubsystem)).withTimeout(1.55),
            new BalanceCommand(pigeonSubsystem, driveSubsystem)
        ).alongWith(new WaitCommand(14.9).andThen(new InstantCommand(()->footSubsystem.footDown(), footSubsystem)))
        );
    }

    public Command getHighCubeMobilzeAuto()
    {
        return(new SequentialCommandGroup(
            new PrintCommand("getHighCubeMobilzeAuto"),
            new armpidCommand(armSubsystem, 0.81,0.6,true,0.1,operator).withTimeout(3).alongWith(
                new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.1), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.25)
            ),
            new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.5), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.2),
            new ParallelCommandGroup(
            new armpidCommand(armSubsystem, 0.60,0.13,true,0.1,operator),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0.3, 0),driveSubsystem)).withTimeout(3.9)),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0, 0),driveSubsystem)).withTimeout(0.8)
            
        ));
    }


    public Command getHighConeMobilizeBalanceAuto()
    {
        return(new SequentialCommandGroup(
            new PrintCommand("getHighConeMobilizeBalanceAuto"),
            new armpidCommand(armSubsystem, 2.4,0,false,0.2,operator).withTimeout(4).andThen(new armpidCommand(armSubsystem, 2.6,0.635,false,0.2,operator).withTimeout(2)).alongWith(
                new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.1), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.25)
            ),
            new WaitCommand(0.5),
            new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.4), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.2),
            new ParallelCommandGroup(
            new armpidCommand(armSubsystem, 0.60,0.05,true,0.1,operator),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(-0.3, 0),driveSubsystem)).withTimeout(4.1)),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0, 0),driveSubsystem)).withTimeout(0.8),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0.4, 0),driveSubsystem)).withTimeout(1.55),
            new BalanceCommand(pigeonSubsystem, driveSubsystem)
        ).alongWith(new WaitCommand(14.9).andThen(new InstantCommand(()->footSubsystem.footDown(), footSubsystem)))
        );

    }

    public Command getHighConeMobilizeAuto()
    {
        return(new SequentialCommandGroup(
            new PrintCommand("getHighConeMobilizeAuto"),
            new armpidCommand(armSubsystem, 2.4,0,false,0.2,operator).withTimeout(3).andThen(new armpidCommand(armSubsystem, 2.6,0.635,false,0.2,operator).withTimeout(2)).alongWith(
                new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.1), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.25)
            ),
            new WaitCommand(0.5),
            new StartEndCommand(
                () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.4), 
                ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
                intakeNewmaticSubsystem).withTimeout(0.2),
            new ParallelCommandGroup(
            new armpidCommand(armSubsystem, 0.60,0.05,true,0.1,operator),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(-0.3, 0),driveSubsystem)).withTimeout(3.9)),
            new RepeatCommand(new InstantCommand(()->driveSubsystem.arcadeDrive(0, 0),driveSubsystem)).withTimeout(0.8)
        ));

    }

    // public Command getAutonomousCommandcopy()
    // {

    //     HashMap<String, Command> eventMap = new HashMap<>();
    //     eventMap.put("Balance", new BalanceCommand(pigeonSubsystem, driveSubsystem));
    //     eventMap.put("marker2", new PrintCommand("marker 2"));
    //     return driveSubsystem.getAutCommandWithEvents(autoPaths.getSelectedTrajectory(), true,eventMap);
    // }

    /**
     * Assign Buttons to Command Triggers
     */
     private void configureTestButtonBindings()
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
         //driver.getBack().whileTrue(new armpidCommand(armSubsystem, -100,0));
         
         driver.getLB().onTrue(new InstantCommand(()-> intakeNewmaticSubsystem.switchIntake(), intakeNewmaticSubsystem));
  
         driver.getUpDpad().whileTrue(new ArmUp(armSubsystem));
         driver.getDownDpad().whileTrue(new ArmDown(armSubsystem));
 
         driver.getY().whileTrue(new ArmInCommand(armSubsystem));
         driver.getX().whileTrue(new ArmOutCommand(armSubsystem));
 
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
        
        //driver.getLeftDpad().whileTrue(new WeightBackCommand(counterweightPIDSubsystem));
        //driver.getRightDpad().whileTrue(new WeightForwardCommand(counterweightPIDSubsystem));

        // When the driver's left bumper is pressed, switch between low and high speed.
        //driver.getLB().whileTrue(new StartEndCommand(() -> armSubsystem.SetActivatePID(true),() -> armSubsystem.SetActivatePID(true), armSubsystem));
        driver.getStart().toggleOnTrue(new BalanceCommand(pigeonSubsystem, driveSubsystem));
        driver.getBack().onTrue(new InstantCommand(()->footSubsystem.toggleFoot() ,footSubsystem));
        
        operator.getRTrigAsButton().whileTrue(new InstantCommand(()-> intakeNewmaticSubsystem.openIntake(), intakeNewmaticSubsystem)).whileFalse(new InstantCommand(()-> intakeNewmaticSubsystem.closeIntake(), intakeNewmaticSubsystem));
        operator.getLB().whileTrue(new StartEndCommand(
            () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(-0.5), 
            ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
            intakeNewmaticSubsystem));

        operator.getLTrigAsButton().whileTrue(new StartEndCommand(
            () -> intakeNewmaticSubsystem.setIntakeMotorSpeed(0.8), 
            ()->intakeNewmaticSubsystem.setIntakeMotorSpeed(0), 
            intakeNewmaticSubsystem).alongWith(new PrintCommand("in")));
            
        //driver.getLTrigAsButton().onTrue(new SwitchSpeed());
        driver.getLTrigAsButton().or(driver.getLB()).whileTrue(new SlowedWhileActiveCommand().andThen(() -> driveSubsystem.stop(), driveSubsystem)); // While left bumper held, slow robot down

        //driver.getX().onTrue(new OpenIntakeCommand(intakeNewmaticSubsystem));
      
        //driver.getLB().whileTrue(new PointToTagCommand(limelightSubsytem, driveSubsystem));
        //driver.getRB().whileTrue(new SyncLimelightPose(limelightSubsytem, driveSubsystem));
        
        //RamseteCommand drivRamseteCommand = driveSubsystem.getRamseteCommand(driveSubsystem.getPose(), RobotState.getSavedPoint(), driveSubsystem);
        //driver.getBack().onTrue(new RavisRamseteCommand(driveSubsystem));


        driver.getY().whileTrue(new armpidCommand(armSubsystem, 0.81,0.595,true,0,operator));


        operator.getY().whileTrue(new armpidCommand(armSubsystem, 0.81,0.595,true,0,operator));
        operator.getB().whileTrue(new armpidCommand(armSubsystem, 0.60,0.13,true,0,operator));
        operator.getA().whileTrue(new armpidCommand(armSubsystem, 0.2096,0.458,false,0,operator));

        operator.getX().whileTrue(new armpidCommand(armSubsystem, 2.4,0,false,0.2,operator).andThen(new armpidCommand(armSubsystem, 2.6,0.635,false,0,operator)));

       // operator.getY().whileTrue(new SequentialCommandGroup( new armpidCommand(armSubsystem, armSubsystem.getRotEncoderPos(),0,false,0.05)));

       operator.getUpDpad().whileTrue(new InstantCommand(()->armSubsystem.changeRotOffset(0.01),armSubsystem));
       operator.getDownDpad().whileTrue(new InstantCommand(()->armSubsystem.changeRotOffset(-0.01),armSubsystem));

       operator.getRightDpad().whileTrue(new InstantCommand(()->armSubsystem.changeTeleOffset(0.01),armSubsystem));
       operator.getLeftDpad().whileTrue(new InstantCommand(()->armSubsystem.changeTeleOffset(-0.01),armSubsystem));

       operator.getStart().onTrue(new InstantCommand(()->armSubsystem.resetArm(),armSubsystem));

       operator.getBack().whileTrue(new LEDOnWhileActive());

       operator.getLS().whileTrue(new RepeatCommand(new InstantCommand(()->armSubsystem.rotateArm(armSubsystem.getArmTorque()*0.005), armSubsystem)));




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
