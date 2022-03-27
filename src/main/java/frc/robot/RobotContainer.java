// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoHood;
import frc.robot.commands.AutoMidClimb;
import frc.robot.commands.AutoShootLeaveTarmac;
import frc.robot.commands.AutoTwoBall;
import frc.robot.commands.CollectGroup;
import frc.robot.commands.CommandHood;
import frc.robot.commands.CommandShooter;
import frc.robot.commands.Drive;
import frc.robot.commands.JoystickClimb;
import frc.robot.commands.LeaveTarmack;
import frc.robot.commands.MoveTower;
import frc.robot.commands.PrimitiveShooter;
import frc.robot.commands.TargetTracking;
import frc.robot.commands.paths.Threeball;
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
  // private final Dashboard m_dashboard = new Dashboard(m_base, m_collector, m_shooter, null);

  Trajectory trajectory = new Trajectory();
  //ShuffleboardTab driverDashboard = Shuffleboard.getTab("Dashboard");
  SendableChooser<SelectableTrajectory> selectedPath = new SendableChooser<SelectableTrajectory>();

  private final TargetPackage lowPackage = new TargetPackage(
    ShooterConstants.kGains.kP,
    ShooterConstants.kGains.kF,
    HoodConstants.HUB_LOW_SHOT_COUNT,
    ShooterConstants.SHOOTER_POWER_HUB_LOW
  );

  private final TargetPackage highPackage = new TargetPackage(
    ShooterConstants.kGains.kP,
    ShooterConstants.kGains.kF,
    HoodConstants.HUB_SHOT_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_HUB_HIGH
  );

  private final TargetPackage tarmacPackage = new TargetPackage(
    ShooterConstants.kGainsTarmac.kP,
    ShooterConstants.kGainsTarmac.kF,
    HoodConstants.TARMAC_SHOT_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_TARMAC_HIGH
  );

  private final TargetPackage defualtPackage = new TargetPackage(
    ShooterConstants.kGains.kP,
    ShooterConstants.kGains.kF,
    HoodConstants.TARMAC_SHOT_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_TARMAC_HIGH
  );

  private final TargetPackage twoBallTargetPackage = new TargetPackage(
    ShooterConstants.SHOOTER_Kp_AUTO_TWO_BALL,
    ShooterConstants.SHOOTER_FF_AUTO_TWO_BALL,
    HoodConstants.AUTO_SHOT_TWOBALL_TICK_COUNT,
    ShooterConstants.SHOOTER_POWER_TARMAC_HIGH
  );

  private final SelectableTrajectory LeaveTarmac = new SelectableTrajectory(
    "Leave Tarmac",
    new LeaveTarmack(m_base)
  );

  private final SelectableTrajectory ShootLeaveTarmac = new SelectableTrajectory(
    "Auto Shoot Leave Tarmac",
    new AutoShootLeaveTarmac(m_base, m_shooter, m_hood, m_collector)
  );

  private final SelectableTrajectory ShootLeaveTarmacCollectShoot = new SelectableTrajectory(
    "Two Ball Auto",
    new AutoTwoBall(
      m_base,
      m_shooter,
      m_collector,
      m_hood,
      twoBallTargetPackage
    )
  );

  private final SelectableTrajectory ThreeBallAuto = new SelectableTrajectory(
    "Auto Three Ball",
    new Threeball(m_base, m_shooter, m_collector, m_limelight, m_hood)
  );

  private final SelectableTrajectory TwoBallBlue = new SelectableTrajectory(
    "Two Ball Blue",
    "output/2 Ball Blue.wpilib.json"
  );

  // private final SelectableTrajectory autoMidClimb = new SelectableTrajectory(
  //   "AutoMidClimb",
  //   new AutoMidClimb(m_base, m_climbers)
  // );

  /**
   * This contains all the trajectories that can be selected from the dashboard.
   */
  private final SelectableTrajectory[] trajectories = {
    LeaveTarmac,
    ShootLeaveTarmac,
    ShootLeaveTarmacCollectShoot,
    ThreeBallAuto,
    // autoMidClimb,
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    for (SelectableTrajectory t : trajectories) {
      if (t.location == ShootLeaveTarmac.name) {
        selectedPath.setDefaultOption(t.name, t);
      } else {
        selectedPath.addOption(t.name, t);
      }
    }

    SmartDashboard.putData(selectedPath);

    // for now select leave blue 1 for testing

    //Initialize and publish for the first time.  Default command of dashboard handles thereafter.
    //m_dashboard.initialize();
    //m_dashboard.publishDashboard();
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_VELOCITY,
      this.m_shooter.getMotorOutputPercent()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_TARGET_RPMS,
      ShooterConstants.SHOOTER_POWER_HUB_HIGH
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_RPMS,
      m_shooter.getMotorRpms()
    );
    SmartDashboard.putBoolean(Dashboard.DASH_SHOOTER_READY, false);
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_P,
      ShooterConstants.kGains.kP
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_SHOOTER_FF,
      ShooterConstants.kGains.kF
    );

    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_LONG_ARM_POSITION,
      m_climbers.longClimberModule.getPosition()
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_SHORT_ARM_POSITION,
      m_climbers.shortClimberModule.getPosition()
    );
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
    // base controller
    // left joystick

    this.m_base.setDefaultCommand(new Drive(m_base, this.gamepad_base));
    this.m_climbers.setDefaultCommand(
        new JoystickClimb(m_climbers, this.gamepad_tower)
      );
    //this.m_dashboard.setDefaultCommand(new DashboardPublish(m_dashboard));

    SmartDashboard.putNumber(Dashboard.DASH_HOOD_P, HoodConstants.kGains.kP);
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_I, HoodConstants.kGains.kI);
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_D, HoodConstants.kGains.kD);

    SmartDashboard.putNumber(
      Dashboard.DASH_TARGET_TRACKER_KP,
      BaseConstants.targetTrackerGains.kP
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_TARGET_TRACKER_KI,
      BaseConstants.targetTrackerGains.kI
    );
    SmartDashboard.putNumber(
      Dashboard.DASH_TARGET_TRACKER_KD,
      BaseConstants.targetTrackerGains.kD
    );

    SmartDashboard.putNumber(
      Dashboard.DASH_HOOD_TICKS,
      HoodConstants.IDLE_TICK_COUNT
    );
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_OUTPUT, 0);
    SmartDashboard.putNumber(Dashboard.DASH_HOOD_DRAW, 0);
    SmartDashboard.putNumber(
      Dashboard.DASH_CLIMBER_LIMITER,
      ClimberModule.CLIMBER_LIMITER
    );

    //Shooter - Shoot - move tower to push ball up to shooter
    new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_X)
    .whenPressed(
        new MoveTower(
          m_collector,
          ShooterConstants.SHOOTER_POWER_HUB_HIGH,
          Constants.JOYSTICK_BUTTON_X,
          gamepad_tower,
          true
        )
      );
    new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_B)
    .whenPressed(
        new CommandShooter(
          m_shooter,
          m_hood,
          gamepad_tower,
          highPackage,
          tarmacPackage,
          lowPackage,
          defualtPackage,
          m_limelight.getDistanceSupplier(),
          m_limelight.getAngleSupplier(),
          Constants.JOYSTICK_BUTTON_B
        )
      );
    // Collector Intake
    new JoystickButton(gamepad_base, Constants.JOYSTICK_RIGHT_BUMPER)
    .whenPressed(
        new CollectGroup(
          m_collector,
          gamepad_base,
          Constants.JOYSTICK_RIGHT_BUMPER,
          true
        )
      );
    // Collector Outtake
    new JoystickButton(gamepad_base, Constants.JOYSTICK_LEFT_BUMPER)
    .whenPressed(
        new CollectGroup(
          m_collector,
          gamepad_base,
          Constants.JOYSTICK_LEFT_BUMPER,
          false
        )
      );
    new JoystickButton(gamepad_base, Constants.JOYSTICK_BUTTON_A)
    .whenPressed(new TargetTracking(m_base, m_limelight));
    // Climbers
    // new JoystickButton(gamepad_tower, Constants.JOYSTICK_BUTTON_Y)
    // .whenPressed(
    //     new MoveShortArms(
    //       m_climbers,
    //       ClimberModule.SHORT_ARM_MID_CLIMB_START,
    //       MoveShortArms.ARM_MOVE_UP
    //     )
    //     .andThen(
    //         // .andThen(
    //         //     new MoveLongArms(
    //         //       m_climbers,
    //         //       ClimberModule.LONG_ARM_MID_CLIMB_START,s
    //         //       MoveLongArms.ARM_MOVE_UP
    //         //     )
    //         //   )
    //         new ClimbDrive(m_base, m_climbers, 0, 0.4, false)
    //       )
    //   );
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
    if (selectedPath.getSelected() == null) {
      selectedPath.setDefaultOption(ShootLeaveTarmac.name, ShootLeaveTarmac);
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
