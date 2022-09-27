// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Stream;

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

  public List<Vector2d> streamListVector(Stream<Vector2d> stream) {
    Iterator<Vector2d> iterator = stream.iterator();
    ArrayList<Vector2d> list = new ArrayList<>();

    iterator.forEachRemaining((e) -> list.add(e));

    return list;
  }

  /**
   * This will interpolate the value for the y column using input e.
   */
  public double interp(double e) {
    ArrayList<Vector2d> greaterPoints = new ArrayList<>(streamListVector(points.stream().filter(v -> v.x > e)));
    ArrayList<Vector2d> smallerPoints = new ArrayList<>(streamListVector(points.stream().filter(v -> v.x < e)));

    if (greaterPoints.isEmpty() || smallerPoints.isEmpty()) {
      return Double.NaN;
    }

    Vector2d vec1 = smallerPoints.get(0);
    Vector2d vec2 = greaterPoints.get(0);

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
