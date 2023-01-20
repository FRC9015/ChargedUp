// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;

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

    private ShuffleboardTab teleopTab, autoTab, currentTab;
    private ShuffleboardLayout balanceLayout, driveLayout, counterweightLayout;  
    private static ArrayList<String> dataInstances;

    public DriveData drive;
    public BalanceData balance;
    public CounterweightData counterweight;

    private Dashboard() {
        init();
        dataInstances = new ArrayList<String>();
    }

    public void init() {
        initTabs();
        initLayouts();
        initSubclasses();
    }

    public void initTabs() {
        teleopTab = Shuffleboard.getTab(DashboardConstants.TELEOP_TAB_NAME);
        autoTab = Shuffleboard.getTab(DashboardConstants.AUTO_TAB_NAME);
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
    }

    public void initSubclasses() {
        drive = new DriveData(driveLayout);
        balance = new BalanceData(balanceLayout);
        counterweight = new CounterweightData(counterweightLayout);
    }

    public void setCurrentTab(CurrentTab newTab) {
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

    public void putData(String name, Sendable send) {
        // check if the sendable with the given name has already been sent
        int indexExists = dataInstances.indexOf(name);
        if (indexExists < 0) {
            currentTab.add(name, send);
            dataInstances.add(name);
        }
        
    }

    // Nested class that handles all drivebase interactions with the dashboard
    public class DriveData {

        private ShuffleboardLayout layout;
        private static SimpleWidget speedMultiplierSelect, speedMode;


        public DriveData(ShuffleboardLayout myLayout) {
            this.layout = myLayout;

            // Creates a number slider with a min value of 0 and max value of 1;
            if(speedMultiplierSelect == null) speedMultiplierSelect = layout.addPersistent("Speed Multiplier", DriveConstants.SLOW_SPEED_MULTIPLIER)
                    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
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

}
