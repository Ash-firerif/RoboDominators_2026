package frc.robot.subsystems.turret.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    boolean motorConnected = false;
    boolean limitSwitch = false;
    double positionRots = 0.0;
    double velocityRPS = 0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  public static enum TurretIOOutputMode {
    BRAKE,
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public static class TurretIOOutputs {

    public TurretIOOutputMode mode = TurretIOOutputMode.BRAKE;
    // Closed loop control
    public double positionRots = 0.0;
    //public double kP = 0.0;
    //public double kD = 0.0;

    // Open loop control for homing
    public double appliedVolts = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}