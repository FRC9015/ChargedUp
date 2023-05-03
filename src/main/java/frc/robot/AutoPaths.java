package frc.robot;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPaths {
    private static final AutoPaths INSTANCE = new AutoPaths();

    public static AutoPaths getInstance() {
        return INSTANCE;
    }

    //private File pathsDir;
    //private File[] pathFiles;
    private SendableChooser<Command> paths;

    private AutoPaths() {

        paths = new SendableChooser<Command>();
    }

    public void init() {


        //pathNames.add(pathName);
        //pathNames.add(pathName);
        //pathNames.add(pathName);

        paths.addOption("new cone leave balance", RobotContainer.getInstance().newHighConeMobilizeBalance());
        paths.addOption("new cube leave balance", RobotContainer.getInstance().newHighCubeMobilizeBalance());
        paths.addOption("new cone leave intake", RobotContainer.getInstance().newHighConeMobilizeIn());
        paths.addOption("new cube leave intake", RobotContainer.getInstance().newHighCubeMobilizeIn());
        paths.addOption("HighCube", RobotContainer.getInstance().getHighCubeAuto());

        paths.addOption("HighConeMobilizeBalance", RobotContainer.getInstance().getHighConeMobilizeBalanceAuto());
        paths.addOption("HighCubeMobilizeBalance", RobotContainer.getInstance().getHighCubeMobilzeBalanceAuto());
        paths.addOption("HighConeMobilize", RobotContainer.getInstance().getHighConeMobilizeAuto());
        paths.addOption("HighCubeMobilize", RobotContainer.getInstance().getHighCubeMobilzeAuto());
        paths.addOption("HighConeBalance", RobotContainer.getInstance().getHighConeBalanceAuto());
        paths.addOption("HighCubeBalance", RobotContainer.getInstance().getHighCubeBalanceAuto());
        paths.addOption("HighConeMobilizeBalanceWITHHASTE", RobotContainer.getInstance().getHighConeMobilizeBalanceAutoWITHHASTE());

        paths.addOption("**TEST** just new balance", RobotContainer.getInstance().justBalance());
        paths.addOption("**TEST** turn 360", RobotContainer.getInstance().getTurn360());
        paths.addOption("**TEST** drive 1m", RobotContainer.getInstance().getDrive1m());




        
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
