package frc.robot.subsystems.turret.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchCalculator;
import frc.robot.subsystems.turret.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.turret.flywheel.FlywheelIO.FlywheelIOOutputs;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();
  //private boolean atGoal = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @AutoLogOutput
  public boolean isFlywheelOnTarget() {
    double frontError = Math.abs(outputs.frontVelocityRPS - inputs.frontVelocityRPS);
    double backError = Math.abs(outputs.backVelocityRPS - inputs.backVelocityRPS);
    return (frontError < Constants.Turret.FLYWHEEL_ON_TARGET_TOLERANCE_RPS) && (backError < Constants.Turret.FLYWHEEL_ON_TARGET_TOLERANCE_RPS);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel",inputs);
    //outputs.frontVelocityRPS = 0;
    //outputs.backVelocityRPS = 0;
    //outputs.mode = FlywheelIOOutputMode.VELOCITY;

    Logger.recordOutput("Flywheel/OnTarget", isFlywheelOnTarget());
    io.applyOutputs(outputs);
  }

  private void runVelocity(double frontVelocityRPS, double backVelocityRPS) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.frontVelocityRPS = frontVelocityRPS;
    outputs.backVelocityRPS = backVelocityRPS;
    Logger.recordOutput("Flywheel/FrontGoal", frontVelocityRPS);
    Logger.recordOutput("Flywheel/BackGoal", backVelocityRPS);
  }

  private void stop() {
    outputs.frontAppliedVolts = 0;
    outputs.backAppliedVolts = 0;
    outputs.mode = FlywheelIOOutputMode.COAST;
  }

  public Command runTrackTargetCommand() {
  return runEnd(
      () -> 
        {var params = LaunchCalculator.getInstance().getParameters();
        runVelocity(params.frontFlywheelSpeed(),params.backFlywheelSpeed());},
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier frontVelocity, DoubleSupplier backVelocity) {
    return runEnd(() -> runVelocity(frontVelocity.getAsDouble(),backVelocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}