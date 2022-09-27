// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here */
public class LinearInterpolationTable {
  /*
   * To get a better understanding of how linear interpolation works.
   * https://en.wikipedia.org/wiki/Linear_interpolation
   */
  ArrayList<Vector2d> points;
  double lastResult;

  public LinearInterpolationTable(List<Vector2d> points) {
    ArrayList<Vector2d> mutList = new ArrayList<>(points);

    mutList.sort((a, b) -> b.x > a.x ? 1 : 0);

    this.points = mutList;
  }

  /**
   * This will interpolate the value for the y column using input e.
   */
  public double interp(double e) {
    Vector2d vec1 = null;
    Vector2d vec2 = null;

    for (Vector2d vec : points) {
      if (vec.x < e) {
        vec1 = vec;
      }

      if (vec.x > e) {
        vec2 = vec;
      }
    }

    /*
     * If vector two is undefined and vector one is defined then that means the
     * input is bigger then whats in our table, so to make sure the output won't
     * become zero in this situation we will assign the second vector to the biggest
     * vector in our table.
     */
    if (vec2 == null && vec1 != null) {
      vec2 = points.get(points.size() - 1);
    }

    if (vec1 == null || vec2 == null) {
      return 0;
    }

    // https://matthew-brett.github.io/teaching/linear_interpolation.html
    return vec1.y + (e - vec1.x) * ((vec2.y - vec1.y) / (vec2.x - vec1.x));
  }
}
