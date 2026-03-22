// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Drive;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private final DriverTimeNotifications m_notifications = new DriverTimeNotifications();
    private int m_updateTick = 0;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        StatusLogger.disableAutoLogging();
        SmartDashboard.putData(CommandScheduler.getInstance());
        SignalLogger.enableAutoLogging(false);
        if (Drive.comp) Elastic.selectTab("Prematch");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (!Drive.comp) return;

        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("MatchTime", matchTime);

        if (m_updateTick % 10 == 0 && DriverStation.isTeleopEnabled()) {
            m_notifications.update(matchTime);
        }

        m_updateTick++;
    }

    @Override
    public void autonomousInit() {

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        if (Drive.comp) Elastic.selectTab("Autonomous");
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
        if (Drive.comp) Elastic.selectTab("Teleop");
    }

    @Override public void disabledInit() {}
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private class DriverTimeNotifications {

        private final String COLOR_GREEN  = "#148314cc";
        private final String COLOR_RED    = "#FF0000";
        private final String COLOR_YELLOW = "#ffc800ff";

        // Cached from FMS game-specific message; null until available
        private Boolean m_ownAllianceInactive = null;

        private boolean ensureInitialized() {
            if (m_ownAllianceInactive != null) return true;

            String gameData = DriverStation.getGameSpecificMessage(); // first char = inactive alliance
            var alliance = DriverStation.getAlliance();

            if (gameData == null || gameData.isEmpty() || alliance.isEmpty()) return false;

            char ownAllianceChar = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';
            m_ownAllianceInactive = (ownAllianceChar == gameData.charAt(0));
            return true;
        }

        public void update(double time) {
            String hexColor;
            double timeUntilNextShift;

            if (time > 130 || time <= 30) {
                // Initial/end-of-match buffer — always green
                hexColor = COLOR_GREEN;
                timeUntilNextShift = (time > 130) ? time - 130 : Math.max(0, time);
            } else if (!ensureInitialized()) {
                // Shift period, but no FMS data yet
                hexColor = COLOR_YELLOW;
                timeUntilNextShift = -1;
            } else {
                // Shift period (130s–30s): alternate active/inactive in 25s blocks
                int block = (int) ((130 - time) / 25);
                timeUntilNextShift = 25 - ((130 - time) % 25);
                boolean isActive = (block % 2 == 0) != m_ownAllianceInactive;
                hexColor = isActive ? COLOR_GREEN : COLOR_RED;
            }

            SmartDashboard.putString("Hub/StatusColor", hexColor);
            SmartDashboard.putNumber("Hub/ShiftTimer", Math.round(timeUntilNextShift));
        }
    }
}