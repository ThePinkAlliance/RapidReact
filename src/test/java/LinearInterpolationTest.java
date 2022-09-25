import static org.junit.Assert.assertEquals;

import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.LinearInterpolationTable;
import org.junit.Test;

public class LinearInterpolationTest {
  List<Vector2d> points = List.of(
      new Vector2d(100, 105),
      new Vector2d(110, 115),
      new Vector2d(120, 125),
      new Vector2d(130, 135),
      new Vector2d(140, 145),
      new Vector2d(150, 155));

  LinearInterpolationTable table = new LinearInterpolationTable(points);

  @Test
  public void CheckTableOutput() {
    assertEquals(table.interp(123), 128.0, Double.POSITIVE_INFINITY);
  }
}
