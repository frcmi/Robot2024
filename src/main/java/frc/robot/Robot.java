// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.sql.Driver;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private static final PowerDistribution pdh = new PowerDistribution();
  private static final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Status");

  public static final GenericEntry allianceShuffleboardEntry = shuffleboardTab
          .add("Alliance", true)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withPosition(3, 0)
          .withSize(2, 1)
          .withProperties(
                  Map.of(
                          "Color when true", "red",
                          "Color when false", "blue"
                  )
          )
          .getEntry();

  public static final GenericEntry stationShuffleboardEntry = shuffleboardTab
          .add("Station", 0)
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(3, 1)
          .withSize(2, 1)
          .getEntry();

  static {
    shuffleboardTab.add("PDH", pdh)
            .withWidget(BuiltInWidgets.kPowerDistribution)
            .withPosition(0,0)
            .withSize(3,3)
            .withProperties(Map.of("Glyph", "POWER_OFF"));
  }

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    Pose2d currentPose = m_robotContainer.swerveSubsystem.getPose();
    e.setRobotPose(m_robotContainer.gotoAutoThing2);
    SmartDashboard.putData("I want to be at", e);
    SmartDashboard.putNumber("Dist from want", Math.sqrt(Math.pow(m_robotContainer.gotoAutoThing2.getX() - currentPose.getX(), 2) + Math.pow(m_robotContainer.gotoAutoThing2.getY() - currentPose.getY(), 2)));
  
    // TODO: measure if this has a perf impact?
    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // alliance.ifPresent(value -> allianceShuffleboardEntry.setBoolean(value == Alliance.Red));

    // OptionalInt station = DriverStation.getLocation();
    // station.ifPresent(stationShuffleboardEntry::setInteger);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putBoolean("Enabled", false);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public Field2d e = new Field2d();

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putBoolean("Enabled", true);}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
