// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drive;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
		SmartDashboard.putData(CommandScheduler.getInstance());
		if (Drive.comp)
			Elastic.selectTab("Prematch");
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		if (Drive.comp) {
			double matchTime = DriverStation.getMatchTime();
			SmartDashboard.putNumber("MatchTime", matchTime);
			// Try to prime cached FMS/alliance data periodically (non-blocking)
			if (m_updateTick % 10 == 0) {
				MatchInfo.getInstance().ensureInitialized();
			}
			// Update display/notifications every 10 loops (200ms) while teleop; I would move it to teleopPeriodic but then I'd have to call matchTime again or pass it in some other way, and this is cleaner
			if (DriverStation.isTeleopEnabled() && (m_updateTick % 10 == 0)) {
				m_notifications.update(matchTime);
			}
			m_updateTick++;
		}
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
		if (Drive.comp)
			Elastic.selectTab("Autonomous");
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		if(Drive.comp) {
			Elastic.selectTab("Teleop");
		}
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopPeriodic() {
		
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	private class DriverTimeNotifications {

		// Hex Code Constants for Elastic
		private final String COLOR_GREEN = "#148314cc";
		private final String COLOR_RED = "#FF0000";
		private final String COLOR_YELLOW = "#ffc800ff";

		public void update(double time) {
			String hexColor;
			double timeUntilNextShift;

			// 1. Initial/End Match Buffer (Always Green)
			if (time > 130 || time <= 30) {
				hexColor = COLOR_GREEN;
				timeUntilNextShift = (time > 130) ? time - 130 : Math.max(0, time);
			} 
			// 2. Shift Period (130s to 30s)
			else {
				if (!MatchInfo.getInstance().isFmsDataValid()) {
					//no FMS data yet
						hexColor = COLOR_YELLOW;
						timeUntilNextShift = -1; 
				} else {
					// Calculate which 25s block we are in
					int block = (int) ((130 - time) / 25);
					timeUntilNextShift = 25 - ((130 - time) % 25);

					boolean isEvenBlock = (block % 2 == 0);
						boolean isOwnInactive = MatchInfo.getInstance().isOwnAllianceInactive();
						boolean isActive = (isEvenBlock != isOwnInactive);
					
					hexColor = isActive ? COLOR_GREEN : COLOR_RED;
				}
			}

			SmartDashboard.putString("Hub/StatusColor", hexColor);
			SmartDashboard.putNumber("Hub/ShiftTimer", Math.round(timeUntilNextShift));
		}
	}
}