// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.Arm.ArmDefaultControlCommand;
import frc.robot.commands.Arm.ArmPIDCommand;
import frc.robot.commands.Balance.BalanceCommand;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.SlowedWhileActiveCommand;
import frc.robot.commands.Intake.CloseIntakeCommand;
import frc.robot.commands.Intake.ConeIntakeAndHoldCommand;
import frc.robot.commands.Intake.OpenIntakeCommand;
import frc.robot.commands.Intake.RunIntakeMotorCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DiffDriveSubsystem;
import frc.robot.subsystems.FootSubsystem;
import frc.robot.subsystems.IntakePneumaticSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDPreset;
import frc.robot.subsystems.LimelightSubsytem;
import frc.robot.subsystems.PigeonSubsystem;

/**
 * Very little robot logic should actually be handled in the {@link Robot} periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer INSTANCE;

    public static RobotContainer getInstance() {
        if (INSTANCE == null) INSTANCE = new RobotContainer();

        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...
    DiffDriveSubsystem driveSubsystem = DiffDriveSubsystem.getInstance();
    PigeonSubsystem pigeonSubsystem = PigeonSubsystem.getInstance();
    ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    LimelightSubsytem limelightSubsytem = LimelightSubsytem.getInstance();
    FootSubsystem footSubsystem = FootSubsystem.getInstance();
    IntakePneumaticSubsystem intakePneumaticSubsystem = IntakePneumaticSubsystem.getInstance();
    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    public final RobotState robotState = RobotState.getInstance();
    private AutoPaths autoPaths = AutoPaths.getInstance();

    private DriverController driver;
    private OperatorController operator;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        PathPlannerServer.startServer(9015);

        init();

        driveSubsystem.setDefaultCommand(new ArcadeDrive(driver));
        armSubsystem.setDefaultCommand(new ArmDefaultControlCommand(operator));
    }

    public void initRobot() {
        pigeonSubsystem.resetAngles();
        Dashboard.getInstance().putSendable("RobotState", robotState);

        init();

        autoPaths.init();

        Dashboard.getInstance().addAutoPathChooser(autoPaths.getCommandAutos());
    }

    private void init() {
        if (driver == null) driver = new DriverController(new XboxController(0));
        Dashboard.getInstance().putSendable("Driver", driver);
        if (operator == null) operator = new OperatorController(new XboxController(1));
        Dashboard.getInstance().putSendable("Operator", operator);
    }

    public Command getHighCubeMobilzeBalanceAuto() {
        return (new SequentialCommandGroup(
                        new PrintCommand("getHighCubeMobilzeBalanceAuto"),
                        new ArmPIDCommand(0.81, 0.6, true, 0.1, operator)
                                .withTimeout(3)
                                .alongWith(new StartEndCommand(
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                                intakePneumaticSubsystem)
                                        .withTimeout(0.25)),
                        new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(-0.5),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.2),
                        new ParallelCommandGroup(
                                new ArmPIDCommand(0.60, 0.13, true, 0.1, operator),
                                new RepeatCommand(new InstantCommand(
                                                () -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem))
                                        .withTimeout(4.1)),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                                .withTimeout(0.8),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(-0.4, 0), driveSubsystem))
                                .withTimeout(1.55),
                        new BalanceCommand())
                .alongWith(new WaitCommand(14.9)
                        .andThen(new InstantCommand(() -> footSubsystem.footDown(), footSubsystem))));
    }

    public Command getHighCubeBalanceAuto() {
        return (new SequentialCommandGroup(
                        new PrintCommand("getHighCubeMobilzeBalanceAuto"),
                        new ArmPIDCommand(0.81, 0.6, true, 0.1, operator)
                                .withTimeout(3)
                                .alongWith(new StartEndCommand(
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                                intakePneumaticSubsystem)
                                        .withTimeout(0.25)),
                        new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(-0.5),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.2),
                        new ParallelCommandGroup(
                                new ArmPIDCommand(0.60, 0.13, true, 0.1, operator),
                                new RepeatCommand(new InstantCommand(
                                                () -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem))
                                        .withTimeout(2)),
                        new BalanceCommand())
                .alongWith(new WaitCommand(14.9)
                        .andThen(new InstantCommand(() -> footSubsystem.footDown(), footSubsystem))));
    }

    public Command getHighCubeMobilzeAuto() {
        return (new SequentialCommandGroup(
                new PrintCommand("getHighCubeMobilzeAuto"),
                new ArmPIDCommand(0.81, 0.6, true, 0.1, operator)
                        .withTimeout(3)
                        .alongWith(new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.25)),
                new StartEndCommand(
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(-0.5),
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                intakePneumaticSubsystem)
                        .withTimeout(0.2),
                new ParallelCommandGroup(
                        new ArmPIDCommand(0.60, 0.13, true, 0.1, operator),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem))
                                .withTimeout(3.9)),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                        .withTimeout(0.8)));
    }

    public Command getHighCubeMobilzeIntakeAuto() {
        return (new SequentialCommandGroup(
                new PrintCommand("getHighCubeMobilzeAuto"),
                new ArmPIDCommand(0.81, 0.6, true, 0.1, operator)
                        .withTimeout(3)
                        .alongWith(new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.25)),
                new StartEndCommand(
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(-0.5),
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                intakePneumaticSubsystem)
                        .withTimeout(0.2),
                new ParallelCommandGroup(
                        new ArmPIDCommand(0.40, 0.13, true, 0.1, operator),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem))
                                .withTimeout(3.5)),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0.6), driveSubsystem))
                        .withTimeout(1.38),
                new ArmPIDCommand(0.2096, 0.458, false, 0.15, operator),
                new RepeatCommand(new InstantCommand(
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.2), intakePneumaticSubsystem)
                        .alongWith(new InstantCommand(() -> driveSubsystem.arcadeDrive(0.2, 0))))));
    }

    public Command getHighConeMobilizeBalanceAuto() {
        return (new SequentialCommandGroup(
                        new PrintCommand("getHighConeMobilizeBalanceAuto"),
                        new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                                .withTimeout(4)
                                .andThen(new ArmPIDCommand(2.6, 0.635, false, 0.2, operator).withTimeout(2))
                                .alongWith(new StartEndCommand(
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                                intakePneumaticSubsystem)
                                        .withTimeout(0.25)),
                        new WaitCommand(0.5),
                        new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(-0.4),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.2),
                        new ParallelCommandGroup(
                                new ArmPIDCommand(0.60, 0.05, true, 0.1, operator),
                                new RepeatCommand(new InstantCommand(
                                                () -> driveSubsystem.arcadeDrive(-0.3, 0), driveSubsystem))
                                        .withTimeout(4.1)),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                                .withTimeout(0.8),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0.4, 0), driveSubsystem))
                                .withTimeout(1.55),
                        new BalanceCommand())
                .alongWith(new WaitCommand(14.9)
                        .andThen(new InstantCommand(() -> footSubsystem.footDown(), footSubsystem))));
    }

    public Command getHighConeBalanceAuto() {
        return (new SequentialCommandGroup(
                        new PrintCommand("getHighConeMobilizeBalanceAuto"),
                        new OpenIntakeCommand(),
                        new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                                .withTimeout(4)
                                .andThen(new ArmPIDCommand(2.6, 0.635, false, 0.2, operator).withTimeout(2))
                                .alongWith(new StartEndCommand(
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                                intakePneumaticSubsystem)
                                        .withTimeout(0.25)),
                        new WaitCommand(0.5),
                        new CloseIntakeCommand(),
                        new WaitCommand(1),
                        new ArmPIDCommand(0.60, 0.05, true, 0.1, operator),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(-0.3, 0), driveSubsystem))
                                .withTimeout(2),
                        new BalanceCommand())
                .alongWith(new WaitCommand(14.9)
                        .andThen(new InstantCommand(() -> footSubsystem.footDown(), footSubsystem))));
    }

    public Command getHighConeMobilizeAuto() {
        return (new SequentialCommandGroup(
                new PrintCommand("getHighConeMobilizeAuto"),
                new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                        .withTimeout(3)
                        .andThen(new ArmPIDCommand(2.6, 0.635, false, 0.2, operator).withTimeout(2))
                        .alongWith(new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.25)),
                new WaitCommand(0.5),
                new CloseIntakeCommand(),
                new WaitCommand(1),
                new ArmPIDCommand(0.60, 0.05, true, 0.1, operator),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(-0.4, 0), driveSubsystem))
                        .withTimeout(2.8),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                        .withTimeout(0.8)));
    }

    public Command getHighConeLeftMobilizeAuto() {
        return (new SequentialCommandGroup(
                new PrintCommand("getHighConeMobilizeAuto"),
                new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                        .withTimeout(3)
                        .andThen(new ArmPIDCommand(2.6, 0.635, false, 0.2, operator).withTimeout(2))
                        .alongWith(new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.25)),
                new WaitCommand(0.5),
                new CloseIntakeCommand(),
                new WaitCommand(1),
                new ArmPIDCommand(0.60, 0.05, true, 0.1, operator),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(-0.4, 0), driveSubsystem))
                        .withTimeout(2.8),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                        .withTimeout(0.8)));
    }

    public Command justBalance() {
        return (new BalanceCommand());
    }

    public Command getHighConeMobilizeIntakeAuto() {
        return (new SequentialCommandGroup(
                new PrintCommand("getHighConeMobilizeAuto"),
                new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                        .withTimeout(3)
                        .andThen(new ArmPIDCommand(2.6, 0.635, false, 0.2, operator).withTimeout(2))
                        .alongWith(new StartEndCommand(
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.1),
                                        () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0),
                                        intakePneumaticSubsystem)
                                .withTimeout(0.25)),
                new WaitCommand(0.5),
                new CloseIntakeCommand(),
                new ParallelCommandGroup(
                        new ArmPIDCommand(0.60, 0.05, true, 0.1, operator),
                        new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(-0.3, 0), driveSubsystem))
                                .withTimeout(3.9)),
                new RepeatCommand(new InstantCommand(() -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem))
                        .withTimeout(0.8),
                new ArmPIDCommand(0.2096, 0.458, false, 0.15, operator),
                new RepeatCommand(new InstantCommand(
                                () -> intakePneumaticSubsystem.setIntakeMotorSpeed(0.2), intakePneumaticSubsystem)
                        .alongWith(new InstantCommand(() -> driveSubsystem.arcadeDrive(0.2, 0))))));
    }

    public Command getTurn90() {
        return (new CloseIntakeCommand());
    }

    // public Command getAutonomousCommandcopy()
    // {

    //     HashMap<String, Command> eventMap = new HashMap<>();
    //     eventMap.put("Balance", new BalanceCommand(pigeonSubsystem, driveSubsystem));
    //     eventMap.put("marker2", new PrintCommand("marker 2"));
    //     return driveSubsystem.getAutCommandWithEvents(autoPaths.getSelectedTrajectory(),
    // true,eventMap);
    // }

    /** Assign Buttons to Command Triggers */
    private void configureButtonBindings() {
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

        // When the driver's left bumper is pressed, switch between low and high speed.
        driver.getStart().toggleOnTrue(new BalanceCommand());
        driver.getBack().onTrue(new InstantCommand(() -> footSubsystem.toggleFoot(), footSubsystem));

        driver.getLTrigAsButton()
                .or(driver.getLb())
                .whileTrue(new SlowedWhileActiveCommand()
                        .andThen(
                                () -> driveSubsystem.stop(),
                                driveSubsystem)); // While left bumper held, slow robot down

        // driver.getY().whileTrue(new ArmPIDCommand(0.81, 0.595, true, 0, operator));

        operator.getRTrigAsButton().whileTrue(new ConeIntakeAndHoldCommand()).onFalse(new OpenIntakeCommand());
        operator.getLB().whileTrue(new RunIntakeMotorCommand(-0.5));

        operator.getLTrigAsButton().whileTrue(new RunIntakeMotorCommand(0.8));

        operator.getY().whileTrue(new ArmPIDCommand(0.81, 0.595, true, 0, operator));
        operator.getB().whileTrue(new ArmPIDCommand(0.60, 0.13, true, 0, operator));
        // Intake from ground preset
        operator.getA().whileTrue(new ArmPIDCommand(0.15, 0.3, false, 0, operator));

        operator.getX()
                .whileTrue(new ArmPIDCommand(2.4, 0, false, 0.2, operator)
                        .andThen(new ArmPIDCommand(2.6, 0.635, false, 0, operator)));

        operator.getUpDpad().whileTrue(new InstantCommand(() -> armSubsystem.changeRotOffset(0.01), armSubsystem));
        operator.getDownDpad().whileTrue(new InstantCommand(() -> armSubsystem.changeRotOffset(-0.01), armSubsystem));

        operator.getRightDpad().whileTrue(new InstantCommand(() -> armSubsystem.changeTeleOffset(0.01), armSubsystem));
        operator.getLeftDpad().whileTrue(new InstantCommand(() -> armSubsystem.changeTeleOffset(-0.01), armSubsystem));

        //    operator.getStart().onTrue(new InstantCommand(()->ledSubsystem.pulseColor(new
        // Color(255, 50, 50),2,255,10),ledSubsystem));
        operator.getStart().onTrue(new InstantCommand(() -> ledSubsystem.setPreset(LEDPreset.CONE), ledSubsystem));

        operator.getBack().onTrue(new InstantCommand(() -> ledSubsystem.setPreset(LEDPreset.CUBE), ledSubsystem));

        operator.getLS()
                .whileTrue(new RepeatCommand(new InstantCommand(
                        () -> armSubsystem.rotateArm(armSubsystem.getArmTorque() * 0.005), armSubsystem)));
    }

    public DriverController getDriver() {
        return driver;
    }

    public OperatorController getOperator() {
        return operator;
    }

    /**
     * @return the command to run in teleop
     */
    public Command getTeleopCommand() {
        return null; // null as this is already handled by the drive subsystem's default command
    }
}
