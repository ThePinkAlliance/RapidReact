// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.TargetPackage;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandShooter extends ParallelCommandGroup {

  /** Creates a new CommandShooter. */
  public CommandShooter(
    Shooter m_shooter,
    Hood m_hood,
    Joystick joystick,
    TargetPackage highPackage,
    TargetPackage tarmacPackage,
    TargetPackage lowPackage,
    TargetPackage defualtPackage,
    Supplier<Double> distanceSupplier,
    Supplier<Double> angleSupplier,
    int button_id
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrimitiveShooter(
        m_shooter,
        joystick,
        highPackage,
        tarmacPackage,
        lowPackage,
        defualtPackage,
        distanceSupplier,
        angleSupplier,
        button_id
      ),
      new CommandHood(
        m_hood,
        joystick,
        highPackage,
        tarmacPackage,
        lowPackage,
        defualtPackage,
        distanceSupplier,
        angleSupplier,
        button_id
      )
    );
  }
}
