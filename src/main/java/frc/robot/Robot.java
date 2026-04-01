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
import frc.robot.util.HubSchedule;

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

        public void update(double time) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isEmpty()) {
                SmartDashboard.putString("Hub/Status", COLOR_YELLOW);
                SmartDashboard.putNumber("Hub/ShiftTimer", -1);
                return;
            }

            HubSchedule.HubStatus status = HubSchedule.getHubStatus(alliance.get());
            String color = switch (status.state) {
                case STRICTLY_ACTIVE -> COLOR_GREEN;
                case MARGIN_ACTIVE   -> COLOR_YELLOW;
                case INACTIVE        -> COLOR_RED;
            };

            SmartDashboard.putString("Hub/StatusColor", color);
            SmartDashboard.putNumber("Hub/ShiftTimer", Math.round(status.secsUntilNextShift));
        }
    }
}