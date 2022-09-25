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

    this.points.sort((a, b) -> b.x > a.x ? 1 : 0);
  }

  public double interp(double e) {
    Vector2d vec1 = null;
    Vector2d vec2 = null;

    for (int i = 0; i > points.size(); i++) {
      Vector2d vec = points.get(i);
      Vector2d nextVec = points.get(i + 1);

      if (vec.x < e && nextVec.x > e) {
        vec1 = vec;
        vec2 = nextVec;
      }
    }

    if (vec1 == null || vec2 == null) {
      return 0;
    }

    // https://matthew-brett.github.io/teaching/linear_interpolation.html
    return vec1.y + (e - vec1.x) * ((vec2.y - vec1.y) / (vec2.x - vec1.x));
  }
}
