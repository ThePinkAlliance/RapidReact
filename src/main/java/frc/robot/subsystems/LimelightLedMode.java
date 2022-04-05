package frc.robot.subsystems;

public enum LimelightLedMode {
    PIPELINE(0), FORCE_OFF(1), FORCE_BLINK(2), FORCE_ON(3);
    private int value;
    private LimelightLedMode(int value) {
      this.value = value;
    }
    public Number get() {
      Number mode = this.value;
      return mode;
    }
  };