package frc.robot.subsystems.turret.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.turret.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.turret.hood.HoodIO.HoodIOOutputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();
  private double goalAngle = 0;
  private boolean homing = false;
  private int stallCount = 0;
  private static double offset = 0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @AutoLogOutput
  public boolean isHoodOnTarget() {
    //if (!aimGoal.enable) return true;
    double error = Math.abs(outputs.positionRots - getAngle());
    return error < Constants.Turret.HOOD_ON_TARGET_TOLERANCE_ROT;
  }

  // Commands hood to a specific position
  public void setTarget(double rotations) {
    goalAngle = rotations;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRots")
  public double getAngle() {
    return inputs.positionRots + offset;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood",inputs);
    if(!homing){
      outputs.positionRots = MathUtil.clamp(goalAngle, 0, Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS) - offset;
      outputs.mode = HoodIOOutputMode.BRAKE;
    } else {
      outputs.appliedVolts = Constants.Turret.HOOD_HOMING_VOLTS;
      outputs.mode = HoodIOOutputMode.OPEN_LOOP;
      if(inputs.supplyCurrentAmps >= Constants.Turret.HOOD_HOMING_STALL_CURRENT_AMPS){
        stallCount++;
      } else {
        stallCount = 0;
      }
      if(inputs.limitSwitch || stallCount >= Constants.Turret.HOOD_HOMING_STALL_LOOP_THRESHOLD){
        homing = false;
        offset = -inputs.positionRots;
      }
    }

    Logger.recordOutput("Hood/GoalAngleRots",goalAngle);
    io.applyOutputs(outputs);
  }
}