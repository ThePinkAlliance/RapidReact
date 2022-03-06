// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Collect;
import frc.robot.commands.Drive;
import frc.robot.commands.EnableTower;
import frc.robot.commands.Shoot;
import frc.robot.commands.TestAutoSequential;
import frc.robot.commands.turnTest;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightLedMode;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Tower;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Compressor compressor = new Compressor(
    PneumaticsModuleType.REVPH
  );

  private final Joystick gamepad_base = new Joystick(0);
  private final Base m_base = new Base();
  private final Limelight m_limelight = new Limelight();
  private final Collector m_collector = new Collector();
  // private final Turret m_turret = new Turret();
  private final Shooter m_shooter = new Shooter();
  private final Tower m_tower = new Tower();

  Trajectory trajectory = new Trajectory();
  ShuffleboardTab driverDashboard = Shuffleboard.getTab("Dashboard");
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();

  private final SelectableTrajectory leaveBlueLeft = new SelectableTrajectory(
    "Leave Blue Left",
    new TestAutoSequential(m_base)
  );

  private final SelectableTrajectory turnTest = new SelectableTrajectory(
    "Turn test",
    new turnTest(m_base)
  );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = {
    leaveBlueLeft,
    turnTest,
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    for (SelectableTrajectory t : trajectories) {
      if (t.location == leaveBlueLeft.name) {
        selectedPath.setDefaultOption(t.name, t);
      } else {
        selectedPath.addOption(t.name, t);
      }
    }
    driverDashboard.add(selectedPath);
    // for now select leave blue 1 for testing

    setupDashboardValues();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //base controller
    //left joystick
    this.m_base.setDefaultCommand(new Drive(m_base, this.gamepad_base));
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_A)
    .whenPressed(m_base::zeroGyro);
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_B)
    .whenPressed(new Collect(m_collector, gamepad_base, true));
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_X)
    .whenPressed(new Shoot(m_shooter, 1.0, gamepad_base));
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_Y)
    .whenPressed(new EnableTower(m_tower, 1.0, gamepad_base, true));
 
  }

  public void selectTrajectory(SelectableTrajectory selectableTrajectory) {
    this.trajectory = selectableTrajectory.trajectory;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (selectedPath.getSelected() == null) {
      selectedPath.setDefaultOption("Leave Blue Left", leaveBlueLeft);
    }

    return selectedPath.getSelected().getDefualtCommand();
  }

  public void enableLimelight() {
    if (m_limelight != null) {
      m_limelight.setLedState(LimelightLedMode.FORCE_ON);
    }
  }

  public void disableLimelight() {
    if (m_limelight != null) {
      m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
    }
  }

  public void setupDashboardValues() {
     SmartDashboard.putNumber(Constants.DASH_SHOOTER_POWER, Shooter.SHOOTER_POWER_DEFAULT);
  }
}
