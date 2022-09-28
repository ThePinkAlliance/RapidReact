import static org.junit.Assert.assertEquals;

import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.LinearInterpolationTable;
import org.junit.Test;

public class LinearInterpolationTest {
  /**
   * NOTE: If the signs of the vectors in the list are opposites then the
   * interpolated result will be skewed.
   */
  List<Vector2d> points = List.of(
      new Vector2d(120, 125),
      new Vector2d(110, 115),
      new Vector2d(100, 105),
      new Vector2d(130, 135),
      new Vector2d(140, 145),
      new Vector2d(150, 155));
  List<Vector2d> emptyPoints = List.of(
      new Vector2d(0, 0),
      new Vector2d(0, 0));

  LinearInterpolationTable table = new LinearInterpolationTable(points);
  LinearInterpolationTable emptyTable = new LinearInterpolationTable(emptyPoints);

  @Test
  public void CheckTableOutput() {
    points.iterator().forEachRemaining((vec) -> {
      System.out.println("Vector2d(" + vec.x + "," + vec.y + ")");
    });

    assertEquals(128.0, table.interp(123), 0);
  }

  @Test
  public void MaxTableInput() {
    assertEquals(Double.NaN, table.interp(194), 0);
  }

  @Test
  public void CheckEmptyTable() {
    assertEquals(Double.NaN, emptyTable.interp(20), 0);
  }
}
