// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private Command m_testCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    setUseTiming(isReal()); // Run as fast as possible during replay
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables
                                                                   // are logged by default).
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be
                                                                                 // selected automatically)
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in
                                                                       // Advantage Scope.
    } else {
      String path = ByteLogReplay.promptForPath(); // Prompt the user for a file path on the command line
      Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay

      /**
       * replay results to a new log with the "_sim" suffix
       */
      Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                  // be added.

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (m_robotContainer != null) {
      m_robotContainer.disableLimelight();
      m_robotContainer.resetHood();
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (m_robotContainer != null) {
      m_robotContainer.enableLimelight();
      m_robotContainer.resetHoodEncoder();

      Constants.isRed = NetworkTableInstance
          .getDefault()
          .getTable("FMSInfo")
          .getEntry("IsRedAlliance")
          .getBoolean(true);
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    if (m_testCommand != null) {
      m_testCommand.cancel();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_robotContainer != null) {
      m_robotContainer.enableLimelight();
      m_robotContainer.resetHoodEncoder();
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (m_testCommand != null) {
      m_testCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Resetting all the network enties used for subsystem readiness checks.
    m_robotContainer.resetTestEntries();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // Disabling LiveWindow reenables the scheduler.
    LiveWindow.setEnabled(false);

    m_testCommand = m_robotContainer.getTestCommand();

    if (m_testCommand != null) {
      m_testCommand.schedule();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    if (m_robotContainer != null) {
      // m_robotContainer.disableLimelight();
    }
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
