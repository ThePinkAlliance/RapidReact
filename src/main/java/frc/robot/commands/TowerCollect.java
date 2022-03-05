package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Tower;

public class TowerCollect extends CommandBase {

  Tower m_tower;
  Collector m_collectorSub;
  boolean isRed = Constants.isRed;

  public TowerCollect(Tower towerSubsystem, Collector collectorSubsystem) {
    m_tower = towerSubsystem;
    m_collectorSub = collectorSubsystem;

    addRequirements(collectorSubsystem);
    addRequirements(towerSubsystem);

    this.m_collectorSub = collectorSubsystem;
    this.m_tower = towerSubsystem;
  }

  @Override
  public void initialize() {
    this.m_collectorSub.SetSpeed(Collector.COLLECTOR_MOTOR_FULL_SPEED);
    this.m_tower.getRed();
    this.m_tower.getBlue();
  }

  @Override
  public void execute() {
    double threshold = Tower.RGB_THRESHOLD;
    if (isRed == true) {
      if (this.m_tower.getBlue() >= threshold) {
        this.m_collectorSub.SetSpeed(20.0);
      }
      if (this.m_tower.getRed() >= threshold) {
        this.m_collectorSub.SetSpeed(0.0); // can change
      }
    }
    if (isRed == false) {
      if (this.m_tower.getBlue() >= threshold) {
        this.m_collectorSub.SetSpeed(0.0);
      }
      if (this.m_tower.getRed() >= threshold) {
        this.m_collectorSub.SetSpeed(20.0); // can change
      }
    }
  }
}
