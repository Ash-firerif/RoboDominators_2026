package frc.robot.subsystems.turret.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    boolean frontMotorConnected = false;
    boolean backMotorConnected = false;
    double frontVelocityRPS = 0.0;
    double backVelocityRPS = 0.0;
    double frontAppliedVolts = 0.0;
    double backAppliedVolts = 0.0;
    double frontCurrentAmps = 0.0;
    double backCurrentAmps = 0.0;
    double frontTempCelsius = 0.0;
    double backTempCelsius = 0.0;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY
  }

  public static class FlywheelIOOutputs {

    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    // Closed loop control
    public double frontVelocityRPS = 0.0;
    public double backVelocityRPS = 0.0;
    //public double kP = 0.0;

    // Open loop
    public double frontAppliedVolts = 0.0;
    public double backAppliedVolts = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void applyOutputs(FlywheelIOOutputs outputs) {}
}