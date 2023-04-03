package frc.robot;

import java.io.File;
import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class AutoPaths {
    private static final AutoPaths INSTANCE = new AutoPaths();

    public static AutoPaths getInstance() {
        return INSTANCE;
    }

    //private File pathsDir;
    //private File[] pathFiles;
    private ArrayList<String> pathNames;
    private SendableChooser<Command> paths;

    private AutoPaths() {

        paths = new SendableChooser<Command>();
    }

    public void init() {


        //pathNames.add(pathName);
        //pathNames.add(pathName);
        //pathNames.add(pathName);

       
        paths.addOption("HighConeMobilizeBalance", RobotContainer.getInstance().getHighConeMobilizeBalanceAuto());
        paths.addOption("HighCubeMobilizeBalance", RobotContainer.getInstance().getHighCubeMobilzeBalanceAuto());
        paths.addOption("HighConeMobilize", RobotContainer.getInstance().getHighConeMobilizeAuto());
        paths.addOption("HighCubeMobilize", RobotContainer.getInstance().getHighCubeMobilzeAuto());
        paths.addOption("turn 90", RobotContainer.getInstance().getTurn90());

        
    }



    public SendableChooser<Command> getChooser() {
        return paths;
    }

    /**
     * 
     * @return instance of a {@link PathPlannerTrajectory} that was selected on the dashboard
     */
    public Command getSelectedTrajectory(){
        
           return (paths.getSelected());
    
    }
}
