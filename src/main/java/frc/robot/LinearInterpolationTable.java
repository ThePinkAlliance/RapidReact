// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here */
public class LinearInterpolationTable {
  /*
   * To get a better understanding of how linear interpolation works.
   * https://en.wikipedia.org/wiki/Linear_interpolation
   */
  List<Vector2d> points;

  public LinearInterpolationTable(List<Vector2d> points) {
    this.points = points;
  }

  private double interp(double e, Vector2d a, Vector2d b) {
    // https://matthew-brett.github.io/teaching/linear_interpolation.html
    return a.y + (e - a.x) * ((b.y - a.y) / (b.x - a.x));
  }
}
