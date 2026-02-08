// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Drive;
import frc.robot.Constants.Vision;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;


  private final DriverTimeNotifications m_notifications = new DriverTimeNotifications();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    LimelightHelpers.setCameraPose_RobotSpace(
        Vision.camName, Vision.camX, Vision.camY, Vision.camZ,
        Vision.camRoll, Vision.camPitch, Vision.camYaw);
        
    if(Drive.comp) Elastic.selectTab("Prematch");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    double matchTime = DriverStation.getMatchTime();
    if(Drive.comp) {
        SmartDashboard.putNumber("Match Time", matchTime);
        
        if (DriverStation.isTeleopEnabled()) {
            m_notifications.update(matchTime);
        }
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    if(Drive.comp) Elastic.selectTab("Autonomous");
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // Cache alliance and FMS data once when teleop starts
    m_notifications.initialize();

    if(Drive.comp) Elastic.selectTab("Teleop");
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

  @Override public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }




private class DriverTimeNotifications {
    private char m_inactiveFirst = ' ';
    private char m_ownAlliance = ' ';
    private boolean m_fmsDataValid = false;

    public void initialize() {
        m_fmsDataValid = false;
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData != null && !gameData.isEmpty()) {
            m_inactiveFirst = gameData.toUpperCase().charAt(0);
            m_fmsDataValid = true;
        }

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_ownAlliance = (alliance.get() == DriverStation.Alliance.Red) ? 'R' : 'B';
        }
    }

    public void update(double time) {
        if (time <= 0) {
            SmartDashboard.putBoolean("Hub/IsActive", false);
            SmartDashboard.putNumber("Hub/Timer", 0.0);
            return;
        }

        boolean hubIsActive = true; 
        double timeUntilNextShift = 0.0;

        // 2026 REBUILT Timing Blocks
        if (time > 130) { 
            hubIsActive = true;
            timeUntilNextShift = time - 130;
        } else if (time > 105) { 
            hubIsActive = !m_fmsDataValid || (m_ownAlliance != m_inactiveFirst);
            timeUntilNextShift = time - 105;
        } else if (time > 80) { 
            hubIsActive = !m_fmsDataValid || (m_ownAlliance == m_inactiveFirst);
            timeUntilNextShift = time - 80;
        } else if (time > 55) { 
            hubIsActive = !m_fmsDataValid || (m_ownAlliance != m_inactiveFirst);
            timeUntilNextShift = time - 55;
        } else if (time > 30) { 
            hubIsActive = !m_fmsDataValid || (m_ownAlliance == m_inactiveFirst);
            timeUntilNextShift = time - 30;
        } else { 
            hubIsActive = true; // Endgame
            timeUntilNextShift = time;
        }

        // Push to Dashboard
        SmartDashboard.putBoolean("Hub/IsActive", hubIsActive);
        // Rounding to 1 decimal place for a cleaner countdown
        SmartDashboard.putNumber("Hub/Timer", Math.round(timeUntilNextShift * 10.0) / 10.0);
    }
  }
}