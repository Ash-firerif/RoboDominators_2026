package frc.robot.subsystems.turret.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.turret.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.turret.flywheel.FlywheelIO.FlywheelIOOutputs;

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
  private boolean isFlywheelOnTarget() {
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
}