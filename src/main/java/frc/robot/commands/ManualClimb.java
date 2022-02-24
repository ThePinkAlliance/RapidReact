// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ClimberModule.SOLENOID_STATE;
import frc.robot.subsystems.Climbers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualClimb extends CommandBase {

  private Climbers m_climbers;
  private DoubleSupplier power;
  private BooleanSupplier inOrOut;

  enum engagedSides {
    IN,
    OUT,
  }

  engagedSides currentEngagedSides = engagedSides.OUT;

  /** Creates a new ManualClimb. */
  public ManualClimb(
    Climbers m_climbers,
    DoubleSupplier power,
    BooleanSupplier inOrOut
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_climbers = m_climbers;
    this.power = power;
    this.inOrOut = inOrOut;

    addRequirements(m_climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbers.setAllSolenoid(SOLENOID_STATE.UNLOCKED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (inOrOut.getAsBoolean()) {
      currentEngagedSides = engagedSides.IN;
    } else if (!inOrOut.getAsBoolean()) {
      currentEngagedSides = engagedSides.OUT;
    }

    switch (currentEngagedSides) {
      case IN:
        m_climbers.commandAllPower(
          0,
          power.getAsDouble(),
          0,
          power.getAsDouble(),
          0.4 // power limit
        );
        break;
      case OUT:
        m_climbers.commandAllPower(
          power.getAsDouble(),
          0,
          power.getAsDouble(),
          0,
          0.4 // power limit
        );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
