package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.C;

/* from team 422 */
/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private double defaultValue;
  private double lastHasChangedValue = defaultValue;

  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  public double getDefault() { return defaultValue; }

  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (C.tuningMode) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key,
                               SmartDashboard.getNumber(key, defaultValue));
    } else {
      SmartDashboard.clearPersistent(key);
    }
  }

  public double get() {
    return C.tuningMode ? SmartDashboard.getNumber(key, defaultValue)
                        : defaultValue;
  }

  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}
