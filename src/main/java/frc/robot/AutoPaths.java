package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DiffDriveSubsystem;
import java.io.File;

public class AutoPaths {
    private static final AutoPaths INSTANCE = new AutoPaths();

    public static AutoPaths getInstance() {
        return INSTANCE;
    }

    private File pathsDir;
    private File[] pathFiles;
    private ArrayList<String> pathNames;
    private SendableChooser<PathPlannerTrajectory> paths;
    private boolean pathsInitialized = false;

    private SendableChooser<Command> commandAutos;

    private AutoPaths() {
        pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner");
        pathNames = new ArrayList<String>();

        commandAutos = new SendableChooser<Command>();
        paths = new SendableChooser<PathPlannerTrajectory>();
    }

    public void init() {
        commandAutos.addOption(
                "HighConeMobilizeBalance", RobotContainer.getInstance().getHighConeMobilizeBalanceAuto());
        commandAutos.addOption(
                "HighCubeMobilizeBalance", RobotContainer.getInstance().getHighCubeMobilzeBalanceAuto());
        commandAutos.addOption("HighConeMobilize", RobotContainer.getInstance().getHighConeMobilizeAuto());
        commandAutos.addOption("HighCubeMobilize", RobotContainer.getInstance().getHighCubeMobilzeAuto());
        commandAutos.addOption("HighConeBalance", RobotContainer.getInstance().getHighConeBalanceAuto());
        commandAutos.addOption("HighCubeBalance", RobotContainer.getInstance().getHighCubeBalanceAuto());
        commandAutos.addOption(
                "**HighCubeMobilizeIntake", RobotContainer.getInstance().getHighCubeMobilzeBalanceAuto());
        commandAutos.addOption(
                "**HighConeMobilizeIntake", RobotContainer.getInstance().getHighConeMobilizeIntakeAuto());
        commandAutos.addOption(
                "**TEST** just balance", RobotContainer.getInstance().justBalance());
        commandAutos.addOption(
                "**TEST** openIntake", RobotContainer.getInstance().getTurn90());
        commandAutos.addOption(
                "**TEST** Drive 12in",
                DiffDriveSubsystem.getInstance().getDriveDistanceCommand(Units.inchesToMeters(6)));
    }

    public SendableChooser<Command> getCommandAutos() {
        return commandAutos;
    }

    /**
     * @return instance of a {@link Command Command Auto} that was selected on the dashboard
     */
    public Command getSelectedAuto() {
        return commandAutos.getSelected();
    }

    public SendableChooser<PathPlannerTrajectory> getPaths() {
        if (!pathsInitialized) throw new Error("Paths not initialized!");
        return paths;
    }

    /**
     * @return instance of a {@link PathPlannerTrajectory} that was selected on the dashboard
     */
    public PathPlannerTrajectory getSelectedTrajectory() {
        if (!pathsInitialized) throw new Error("Paths not initialized!");
        return paths.getSelected();
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(name, DriveConstants.kPathConstraints);

        if (traj == null) {
            return PathPlanner.loadPath(pathNames.get(0), DriveConstants.kPathConstraints);
        } else {
            return traj;
        }
    }

    /** Load pathplanner trajectories from filesystem */
    @SuppressWarnings("unused")
    private void loadPathFiles() {
        pathFiles = pathsDir.listFiles();

        for (File pathFile : pathFiles) {
            String pathFileName = pathFile.getName();

            if (pathFileName.indexOf(".path") >= 0) {
                String pathName = pathFileName.substring(0, pathFileName.indexOf(".path"));

                pathNames.add(pathName);
            }
        }

        for (String pathName : pathNames) {
            PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, DriveConstants.kPathConstraints);

            paths.addOption(pathName, traj);
        }

        pathsInitialized = true;
    }
}
