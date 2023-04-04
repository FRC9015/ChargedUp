// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;

/** Singleton Dashboard Class */
public class Dashboard {

    private static Dashboard INSTANCE = new Dashboard();

    public static Dashboard getInstance() {
        return INSTANCE;
    }

    public enum CurrentTab {
        Auto,
        TeleOp
    }

    private ShuffleboardTab teleopTab, autoTab, currentTab, debugTab;
    private ShuffleboardLayout balanceLayout, driveLayout, counterweightLayout, autoPathLayout; 
    private SimpleWidget intakeOpen, footDown;
    private static SendableChooser<Command> autoPathChooser; 
    private static ArrayList<String> dataInstances;

    public DriveData drive;
    public BalanceData balance;
    public CounterweightData counterweight;
    public AutoPathData autoPath;

    private Dashboard() {
        init();
        dataInstances = new ArrayList<String>();
    }

    public void periodic() {
        intakeOpen.getEntry().setBoolean((RobotState.getIntakeOpen()));
        footDown.getEntry().setBoolean((RobotState.isFeetDown()));
    }

    public void init() {
        initTabs();
        initLayouts();
        initSubclasses();

        try {
            intakeOpen = teleopTab.add("Intake Open", false).withWidget(BuiltInWidgets.kBooleanBox);
            footDown = teleopTab.add("Foot Down", false).withWidget(BuiltInWidgets.kBooleanBox);
        } catch (Error e) {
            e.printStackTrace();
        }
    }

    public void initTabs() {
        teleopTab = Shuffleboard.getTab(DashboardConstants.TELEOP_TAB_NAME);
        autoTab = Shuffleboard.getTab(DashboardConstants.AUTO_TAB_NAME);
        debugTab = Shuffleboard.getTab("Debug");

        currentTab = teleopTab;
    }

    public void initLayouts() {
        balanceLayout = currentTab.getLayout(DashboardConstants.BALANCE_LAYOUT_NAME, BuiltInLayouts.kList)
                .withSize(1, 4).withPosition(2, 0);
        // Create a List layout for drivetrain information on the Teleop Tab
        // Drive data is only needed during Teleop
        driveLayout = teleopTab.getLayout(DashboardConstants.DRIVE_LAYOUT_NAME, BuiltInLayouts.kList).withSize(2, 4)
                .withPosition(0, 0);
        counterweightLayout = teleopTab.getLayout(DashboardConstants.COUNTERWEIGHT_LAYOUT_NAME, BuiltInLayouts.kList).withSize(1, 2)
        .withPosition(3, 0);

        autoPathLayout = autoTab.getLayout(DashboardConstants.AUTO_PATH_LAYOUT_NAME, BuiltInLayouts.kList).withSize(1, 1);
    }
    
    public void initSubclasses() {
        drive = new DriveData(driveLayout);
        balance = new BalanceData(balanceLayout);
        counterweight = new CounterweightData(counterweightLayout);
        autoPath = new AutoPathData(autoPathLayout);
    }

    /**
     * Change the currently selected tab on the dashboard <p>
     * IMPORTANT: This method only switches tabs if we are in a match. This is detected by checking if the FMS is attached.
     * @param newTab The tab to switch to, represented by the {@link CurrentTab} enum
     */
    public void setCurrentTab(CurrentTab newTab) {
        // If the FMS is NOT attached, then we should not switch tabs
        if (DriverStation.isFMSAttached() == false) return;


        if (newTab == CurrentTab.TeleOp) {
            currentTab = teleopTab;
            Shuffleboard.selectTab(DashboardConstants.TELEOP_TAB_NAME);
        } else if (newTab == CurrentTab.Auto) {
            currentTab = autoTab;
            Shuffleboard.selectTab(DashboardConstants.AUTO_TAB_NAME);
        }
        
        // Refresh layouts when current tab is switched
        initLayouts();
        initSubclasses();
    }

    /**
     * Add a {@link Sendable} object to the debug dashboard
     * @param name
     * @param send
     */
    public void putSendable(String name, Sendable send) {
        // check if the sendable with the given name has already been sent
        int indexExists = dataInstances.indexOf(name);
        if (indexExists < 0) {
            //debugTab.add(name, send);
            dataInstances.add(name);
        }
        
    }

    public void addAutoPathChooser(SendableChooser<Command> chooser) {
        if (autoPathChooser == null) {autoTab.add(chooser); autoPathChooser = chooser;}
    }

    /**
     * Add a number to the debug dashboard
     * @param name Name of number
     * @param num double value that you would like to show (int vals can be coerced)
     * @return SimpleWidget instance that you use for updating the data via NetworkTables
     */
    public SimpleWidget putNumber(String name, double num) {
        SimpleWidget toReturn = null;
        // check if the sendable with the given name has already been sent
        int indexExists = dataInstances.indexOf(name);
        if (indexExists < 0) {
            toReturn = debugTab.add(name, num);
            dataInstances.add(name);
        }

        return toReturn;
    }

    /**
     * Dashboard subclass that handles all drive system data and interactions
     */
    public class DriveData {

        private ShuffleboardLayout layout;
        private static SimpleWidget speedMultiplierSelect, speedMode;


        public DriveData(ShuffleboardLayout myLayout) {
            this.layout = myLayout;

            // Creates a number slider with a min value of 0 and max value of 1;
            if(speedMultiplierSelect == null) speedMultiplierSelect = layout.addPersistent("Speed Multiplier", DriveConstants.SLOW_SPEED_MULTIPLIER)
                    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.1, "max", 0.9));
            // Boolean display for whether the drivetrain is running in Slow Mode
            if(speedMode == null) speedMode = layout.add("Slow Mode", true).withWidget(BuiltInWidgets.kBooleanBox);
        }

        /**
         * Get the speed from the slider
         * 
         * @return
         */
        public double getSpeedMultiplier() {
            return speedMultiplierSelect.getEntry().getDouble(DriveConstants.SLOW_SPEED_MULTIPLIER);
        }

        public void setSpeedMode(boolean slowed) {
            speedMode.getEntry().setBoolean(slowed);
        }
    }

    /**
     * Dashboard subclass that displays balance data as well as auto balancing state (balancing using the drive system)
     */
    public class BalanceData {
        private ShuffleboardLayout layout;
        private static SimpleWidget angleDisplay, isBalanced, autoMode;

        public BalanceData(ShuffleboardLayout myLayout) {
            this.layout = myLayout;

            // Dial display for current pitch
            if(angleDisplay == null) angleDisplay = layout.add("Pitch Angle", 0).withWidget(BuiltInWidgets.kDial)
                    .withProperties(Map.of("min", -45, "max", 45));
            if(isBalanced == null) isBalanced = layout.add("Balanced", false).withWidget(BuiltInWidgets.kBooleanBox);
            if(autoMode == null) autoMode = layout.add("Auto Balanced", false).withWidget(BuiltInWidgets.kBooleanBox);
        }

        public void setAngle(double currentAngle) {
            angleDisplay.getEntry().setDouble(currentAngle);
        }

        public void setBalanced(boolean currentlyBalanced) {
            isBalanced.getEntry().setBoolean(currentlyBalanced);
        }

        public void setAutoBalanced(boolean autoBalanced) {
            autoMode.getEntry().setBoolean(autoBalanced);
        }
    }

    /**
     * Displays data related to the counterweight system
     */
    public class CounterweightData {

        private ShuffleboardLayout layout;
        private static SimpleWidget endstop;

        public CounterweightData(ShuffleboardLayout myLayout) {
            layout = myLayout;

            if (endstop == null) endstop = layout.add("Endstop", false).withWidget(BuiltInWidgets.kBooleanBox);
        }

        public void setEndstop(boolean clicked) {
            endstop.getEntry().setBoolean(clicked);
        }
    }

    /**
     * Select and display status for PathPlanner autonomous trajectories
     */
    public class AutoPathData {
        // private ShuffleboardLayout layout;

        public AutoPathData(ShuffleboardLayout myLayout) {
            // this.layout = myLayout;
        }
        
    } 

}
