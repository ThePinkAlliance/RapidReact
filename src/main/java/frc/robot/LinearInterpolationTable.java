// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
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
    /*
     * NOTE:
     * Now after filtering the points list we will have two lists with vectors that
     * are bigger and smaller then our input, now normally after this we would sort
     * each list from greatest to smallest and retrieve the greatest vector of both
     * the newly sorted lists however whether we need to do this depends on our
     * table makeup. For example if we have a table with all positive values then we
     * don't need to worry about sorting however if we have a table with negative
     * values then sorting might become necessary.
     */
    ArrayList<Vector2d> greaterPoints = new ArrayList<>(
        streamListVector(
            points.stream().filter(v -> Math.abs(v.x) > Math.abs(e) && Math.signum(e) == Math.signum(v.x))));
    ArrayList<Vector2d> smallerPoints = new ArrayList<>(
        streamListVector(
            points.stream().filter(v -> Math.abs(v.x) < Math.abs(e) && Math.signum(e) == Math.signum(v.x))));

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
