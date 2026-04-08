package frc.robot.subsystems.turret.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    boolean motorConnected = false;
    boolean limitSwitch = false;
    double positionRots = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum HoodIOOutputMode {
    BRAKE,
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public static class HoodIOOutputs {

    public HoodIOOutputMode mode = HoodIOOutputMode.BRAKE;
    // Closed loop control
    public double positionRots = 0.0;
    //public double kP = 0.0;
    //public double kD = 0.0;

    // Open loop control for homing
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void applyOutputs(HoodIOOutputs outputs) {}
}