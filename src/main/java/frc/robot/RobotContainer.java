// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.util.joystick.JoystickMap;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDoNothing;
import frc.robot.commands.AutoLatchArms;
import frc.robot.commands.AutoShootLeaveTarmac;
import frc.robot.commands.AutoTwoBall;
import frc.robot.commands.CollectGroup;
import frc.robot.commands.CommandHoodTuning;
import frc.robot.commands.CommandShooterTuning;
import frc.robot.commands.Drive;
import frc.robot.commands.JoystickClimb;
import frc.robot.commands.LeaveTarmack;
import frc.robot.commands.LimelightAlign;
import frc.robot.commands.MoveTower;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightLedMode;
import frc.robot.subsystems.Shooter;

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

  private final Joystick gamepad_base = new Joystick(0);
  private final Joystick gamepad_tower = new Joystick(1);
  private final Base m_base = new Base();
  private final Limelight m_limelight = new Limelight();
  private final Collector m_collector = new Collector();
  private final Shooter m_shooter = new Shooter();
  private final Hood m_hood = new Hood();
  private final Climbers m_climbers = new Climbers();

  // DASHBOARD MUST BE LAST SUBSYSTEM INSTANTIATED
  private final Dashboard m_dashboard = new Dashboard(
      m_base,
      m_collector,
      m_shooter,
      m_climbers);

  Trajectory trajectory = new Trajectory();
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();
  private final SelectableTrajectory LeaveTarmac = new SelectableTrajectory(
      "Leave Tarmac",
      new LeaveTarmack(m_base));
  private final SelectableTrajectory ShootLeaveTarmac = new SelectableTrajectory(
      "Auto Shoot Leave Tarmac",
      new AutoShootLeaveTarmac(
          m_base,
          m_shooter,
          m_hood,
          m_collector,
          m_limelight));

  private final SelectableTrajectory doNothing = new SelectableTrajectory("Auto Do Nothing", new AutoDoNothing());

  private final SelectableTrajectory TwoBallAuto = new SelectableTrajectory(
      "Two Ball Auto",
      new AutoTwoBall(m_base, m_shooter, m_collector, m_hood, m_limelight));

  // private final SelectableTrajectory autoMidClimb = new SelectableTrajectory(
  // "AutoMidClimb",
  // new AutoMidClimb(m_base, m_climbers)
  // );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = {
      LeaveTarmac,
      ShootLeaveTarmac,
      TwoBallAuto,
      doNothing
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    for (SelectableTrajectory t : trajectories) {
      if (t.location == TwoBallAuto.name) {
        selectedPath.setDefaultOption(t.name, t);
      } else {
        selectedPath.addOption(t.name, t);
      }
    }

    SmartDashboard.putData(selectedPath);
    m_dashboard.publishInitialDashboard(); // DO NOT REMOVE and DO NOT COMMENT OUT

    this.m_base.setDefaultCommand(new Drive(m_base, this.gamepad_base));
    this.m_climbers.setDefaultCommand(
        new JoystickClimb(m_climbers, this.gamepad_tower));
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
    // Shooter - Shoot - move tower to push ball up to shooter
    new JoystickButton(gamepad_tower, JoystickMap.BUTTON_X)
        .whenPressed(
            new MoveTower(
                m_collector,
                ShooterConstants.SHOOTER_POWER_HUB_HIGH,
                JoystickMap.BUTTON_X,
                gamepad_tower,
                true));
    // Shooter - Rev the shooter up to requested rpm
    new JoystickButton(gamepad_tower, JoystickMap.BUTTON_A)
        .whenPressed(
            new CommandShooterTuning(
                m_shooter,
                m_limelight,
                m_hood,
                m_base,
                gamepad_tower,
                m_limelight.getDistanceSupplier(),
                m_limelight.getAngleSupplier(),
                JoystickMap.BUTTON_A));
    // Hood - Commands the hood to a requested position
    new JoystickButton(gamepad_tower, JoystickMap.BUTTON_B)
        .whenPressed(
            new CommandHoodTuning(m_hood, gamepad_tower, JoystickMap.BUTTON_B));
    new JoystickButton(gamepad_tower, JoystickMap.BUTTON_Y).whenPressed(() -> this.m_shooter.playMusic());
    // Collector Intake
    new JoystickButton(gamepad_base, JoystickMap.RIGHT_BUMPER)
        .whenPressed(
            new CollectGroup(
                m_collector,
                gamepad_base,
                JoystickMap.RIGHT_BUMPER,
                true));
    // Collector Outtake
    new JoystickButton(gamepad_base, JoystickMap.LEFT_BUMPER)
        .whenPressed(
            new CollectGroup(
                m_collector,
                gamepad_base,
                JoystickMap.LEFT_BUMPER,
                false));
    // Base - Align the shooter with the hub
    // new JoystickButton(gamepad_base, JoystickMap.BUTTON_A)
    // .whenPressed(
    // new LimelightAlign(
    // m_base,
    // m_limelight,
    // gamepad_base,
    // JoystickMap.BUTTON_A));
  }

  public void selectTrajectory(SelectableTrajectory selectableTrajectory) {
    this.trajectory = selectableTrajectory.trajectory;
  }

  public void resetHoodEncoder() {
    m_hood.resetHoodEncoder();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (selectedPath.getSelected() == null) {
    // selectedPath.setDefaultOption(TwoBallAuto.name, TwoBallAuto);
    // }

    // return selectedPath.getSelected().getDefualtCommand();

    return new InstantCommand();
  }

  public void enableLimelight() {
    if (m_limelight != null) {
      // Save peoples eye.
      m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
    }
  }

  public void disableLimelight() {
    if (m_limelight != null) {
      m_limelight.setLedState(LimelightLedMode.FORCE_OFF);
    }
  }

  public void testInit() {
    m_base.setPodAngles(0);
  }

  public void resetHood() {
    if (m_hood != null) {
      m_hood.setPosition(HoodConstants.IDLE_TICK_COUNT);
      m_hood.disableCloseLoopControl();
    }
  }
}
