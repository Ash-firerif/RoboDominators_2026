package frc.robot.subsystems.turret.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.turret.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.turret.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.SmartLogger;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();
  private double goalAngle = 0;
  private int stallCount = 0;
  private boolean stallGiveUp = false;
  private int turretOnTargetLoops = 0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @AutoLogOutput
  private boolean isTurretOnTarget() {
    double error = Math.abs(outputs.positionRots - inputs.positionRots);
    if (error < Constants.Turret.TURRET_ON_TARGET_TOLERANCE_ROT) {
      turretOnTargetLoops++;
    } else {
      turretOnTargetLoops = 0;
    }
    return turretOnTargetLoops >= Constants.Turret.TURRET_ON_TARGET_SETTLE_LOOPS;
  }

  public void setTarget(double rotations) {
    goalAngle = rotations;
  }

  @AutoLogOutput(key = "Turret/MeasuredAngleRots")
  public double getAngle() {
    return inputs.positionRots;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret",inputs);
    double posErr = Math.abs(outputs.positionRots - inputs.positionRots);
    boolean isStuck = Math.abs(inputs.velocityRPS) < Constants.Turret.TURRET_STALL_VELOCITY_THRESHOLD_RPS
        && posErr > Constants.Turret.TURRET_STALL_ERROR_THRESHOLD_ROT;
    if (isStuck) {
      stallCount ++;
    } else {
      stallCount = 0;
      stallGiveUp = false;
    }

    if (stallCount >= Constants.Turret.TURRET_STALL_LOOP_THRESHOLD && !stallGiveUp) {
      stallGiveUp = true;
      SmartLogger.logConsole("Turret stall timeout — holding current position", "Turret");
    }
    SmartLogger.logReplay("Turret/StallGiveUpActive", stallGiveUp);
    
    //default to pos pid
    if (stallGiveUp){
      outputs.mode = TurretIOOutputMode.BRAKE;
    } else {
      outputs.positionRots = MathUtil.clamp(goalAngle, Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT);
      outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    }
    
    Logger.recordOutput("Turret/GoalAngleRots", goalAngle);
    io.applyOutputs(outputs);
  }
}