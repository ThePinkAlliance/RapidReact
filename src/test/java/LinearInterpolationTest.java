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
  List<Vector2d> negativePoints = List.of(
      new Vector2d(-40, -145),
      new Vector2d(-30, -135),
      new Vector2d(-20,
          -125),
      new Vector2d(-10, -115),
      new Vector2d(0, 105),
      new Vector2d(30, 135),
      new Vector2d(40, 145),
      new Vector2d(50, 155));
  List<Vector2d> emptyPoints = List.of(
      new Vector2d(0, 0),
      new Vector2d(0, 0));

  LinearInterpolationTable table = new LinearInterpolationTable(points);
  LinearInterpolationTable emptyTable = new LinearInterpolationTable(emptyPoints);
  LinearInterpolationTable negativeTable = new LinearInterpolationTable(negativePoints);

  @Test
  public void CheckTableOutput() {
    assertEquals(128.0, table.interp(123), 0);
  }

  @Test
  public void NegativeTableInput() {
    assertEquals(-140, negativeTable.interp(-35), 0);
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
