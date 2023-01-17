// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
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
    private ShuffleboardLayout balanceLayout, driveLayout;

    public DriveData drive;
    public BalanceData balance;

    private Dashboard() {
        init();
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
    }

    public void initSubclasses() {
        drive = new DriveData(driveLayout);
        balance = new BalanceData(balanceLayout);
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

    // Nested class that handles all drivebase interactions with the dashboard
    public class DriveData {

        private ShuffleboardLayout layout;
        private SimpleWidget speedMultiplierSelect, speedMode;

        public DriveData(ShuffleboardLayout myLayout) {
            this.layout = myLayout;

            // Creates a number slider with a min value of 0 and max value of 1;
            speedMultiplierSelect = layout.addPersistent("Speed Multiplier", DriveConstants.SLOW_SPEED_MULTIPLIER)
                    .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1));
            // Boolean display for whether the drivetrain is running in Slow Mode
            speedMode = layout.add("Slow Mode", true).withWidget(BuiltInWidgets.kBooleanBox);
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
        private SimpleWidget angleDisplay, isBalanced, autoMode;

        public BalanceData(ShuffleboardLayout myLayout) {
            this.layout = myLayout;

            // Dial display for current pitch
            angleDisplay = layout.add("Pitch Angle", 0).withWidget(BuiltInWidgets.kDial)
                    .withProperties(Map.of("min", -45, "max", 45));
            isBalanced = layout.add("Balanced", false).withWidget(BuiltInWidgets.kBooleanBox);
            autoMode = layout.add("Auto Balancing", false).withWidget(BuiltInWidgets.kBooleanBox);
        }

        public void setAngle(double currentAngle) {
            angleDisplay.getEntry().setDouble(currentAngle);
        }

        public void setBalanced(boolean currentlyBalanced) {
            isBalanced.getEntry().setBoolean(currentlyBalanced);
        }

        public void setAutoBalance(boolean autoBalanced) {
            autoMode.getEntry().setBoolean(autoBalanced);
        }
    }

}
