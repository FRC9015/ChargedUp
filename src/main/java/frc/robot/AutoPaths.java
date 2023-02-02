package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class AutoPaths {
    private static final AutoPaths INSTANCE = new AutoPaths();

    public static AutoPaths getInstance() {
        return INSTANCE;
    }

    private File pathsDir;
    private File[] pathFiles;
    private ArrayList<String> pathNames;
    private SendableChooser<PathPlannerTrajectory> paths;

    private AutoPaths() {
        pathsDir = new File(Filesystem.getDeployDirectory(), "pathplanner");

        pathNames = new ArrayList<String>();
        paths = new SendableChooser<PathPlannerTrajectory>();
    }

    public void init() {
        pathFiles = pathsDir.listFiles();

        for (File pathFile : pathFiles) {
            String pathFileName = pathFile.getName();

            if (pathFileName.indexOf(".path") >= 0) {
                String pathName = pathFileName.substring(0, pathFileName.indexOf(".path"));

                pathNames.add(pathName);
            }
        }

        for (String pathName: pathNames) {
            PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, DriveConstants.kPathConstraints);

            paths.addOption(pathName, traj);
        }
    }

    public String[] getPathNames() {
        String[] constantPathNames = new String[pathNames.size()];

        return pathNames.toArray(constantPathNames);
    }

    public SendableChooser<PathPlannerTrajectory> getChooser() {
        return paths;
    }

    /**
     * 
     * @return instance of a {@link PathPlannerTrajectory} that was selected on the dashboard
     */
    public PathPlannerTrajectory getSelectedTrajectory(){
        PathPlannerTrajectory selectedTraj = paths.getSelected();
        if (selectedTraj == null) {
            return PathPlanner.loadPath(pathNames.get(0), DriveConstants.kPathConstraints);
        } else {
            return selectedTraj;
        }
    }
}
