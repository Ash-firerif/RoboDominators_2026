// Turret subsystem - flywheel shooter, adjustable hood, and rotating turret base.
// Supports phased enablement from static fixed shots (Phase 1) through on-the-move
// tracking (Phase 4). Advance phases by changing Constants.Turret.CURRENT_PHASE.
//
// HOMING: home() must be called before Phase 2+ tracking is trusted. The turret drives
// slowly CCW until the hall sensor fires, then zeros the encoder.
//
// MOTIONMAGIC TUNING: after homing, call setTurretPositionTarget(turretRotations) to bounce
// the turret between two points and tune kP/kS/kV in Constants via AdvantageScope.
// Watch Turret/TargetMotorRot vs Turret/RotationMotorRot for tracking quality.
//
// FIRE INTERLOCK: the operator must call enableFire() explicitly each match.
// isReadyToShoot() checks: fireEnabled, turret on target, flywheel on target, hood on target.

package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.SmartLogger;

import static edu.wpi.first.units.Units.Volts;

public class TurretSubsystem extends SubsystemBase {
  private final RobotState robotState;
  private final TurretIO io;  
  private final TurretIOInputs inputs = new TurretIOInputs();
  private final TurretSetpoints setpoints = new TurretSetpoints();
  private final TurretAimGoal aimGoal = new TurretAimGoal();
  private final TurretAimGoal providerGoal = new TurretAimGoal();
  private final TurretSetpointGenerator setpointGenerator = new TurretSetpointGenerator();

  private static final double ACTIVE_PERCENT_THRESHOLD = 0.02;

  // Tracking gate — operator must confirm turret is homed before aim pipeline runs.
  // Until this is true, updateAimFromProvider() is a no-op and no motors move.
  private boolean trackingEnabled = false;

  // Settle counter — counts consecutive loops the turret has been within tolerance.
  // isAimed()/isReadyToShoot() require this to reach TURRET_ON_TARGET_SETTLE_LOOPS.
  // Currently 0 (disabled) — enable once beam breaks are working.
  private int turretOnTargetLoops = 0;

  private boolean manualPositionOverride = false; // when true, skip aim pipeline and hold setpoints.turretPositionMotorRotations

  // Hood homing state — hood creeps down until the limit switch fires, then zeroes encoder
  private boolean hoodHoming = false;

  // Chain-jam stall recovery — if the turret hasn't moved toward its target for
  // TURRET_STALL_TIMEOUT_SECS, snap the MM target to current position to stop fighting.
  // Clears automatically once the turret starts moving again (chain frees itself).
  private double stallTimerSecs   = 0.0;
  private boolean stallGiveUpActive = false; // true while holding current position after timeout
  private int hoodHomingStallLoopCount = 0; // fallback stall counter if limit switch fails

  // Last known encoder position — updated each loop after homing, used to restore encoder
  // if the motor controller reboots mid-match (brownout/power loss) while the RIO stays up.
  private double lastKnownPosition = 0.0;
  private static final double POSITION_SAVE_THRESHOLD = 0.05; // only update if moved this many motor rotations

  // SysId routines — one per flywheel motor, used for kV/kS/kA characterization only.
  // Call sysIdFrontQuasistatic/Dynamic or sysIdBackQuasistatic/Dynamic from RobotContainer.
  private final SysIdRoutine sysIdFront;
  private final SysIdRoutine sysIdBack;



  public TurretSubsystem(RobotState robotState, TurretIO io) {
    this.robotState = robotState;
    this.io = io;
    SmartLogger.logConsole("Turret ready (phase: " + Constants.Turret.CURRENT_PHASE + ")", "Turret");

    sysIdFront = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> com.ctre.phoenix6.SignalLogger.writeString("sysid-test-state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage v) -> io.setFlywheelFrontVoltage(v.in(Volts)),
            null, this));

    sysIdBack = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null,
            state -> com.ctre.phoenix6.SignalLogger.writeString("sysid-test-state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Voltage v) -> io.setFlywheelBackVoltage(v.in(Volts)),
            null, this));

    if (!Constants.Turret.REQUIRE_TURRET_FORWARD_CONFIRM) {
      homeForward();
    }
    hoodHoming = true; // always auto-home the hood at boot — safe regardless of turret position
  }

  // Seeds the turret encoder to TURRET_FORWARD_MOTOR_ROT and marks homed — no sweep needed.
  // Safe to call any time the turret is physically pointing forward.
  // In tournament mode this is called automatically at boot. In testing mode, operator
  // confirms with LT+RT+A after visually verifying the turret is forward.
  public void homeForward() {
    io.restoreTurretEncoder(Constants.Turret.TURRET_FORWARD_MOTOR_ROT);
    lastKnownPosition = Constants.Turret.TURRET_FORWARD_MOTOR_ROT;
    SmartLogger.logConsole("Turret zeroed to forward position (" + Constants.Turret.TURRET_FORWARD_MOTOR_ROT + " rot)", "Turret");
  }

  public Command sysIdFrontQuasistatic(SysIdRoutine.Direction dir) { return sysIdFront.quasistatic(dir); }
  public Command sysIdFrontDynamic(SysIdRoutine.Direction dir)     { return sysIdFront.dynamic(dir); }
  public Command sysIdBackQuasistatic(SysIdRoutine.Direction dir)  { return sysIdBack.quasistatic(dir); }
  public Command sysIdBackDynamic(SysIdRoutine.Direction dir)      { return sysIdBack.dynamic(dir); }

  // ---- Fire interlock ----

  // Allows the aim pipeline to start running. Called once after the lockout confirm.
  public void enableTracking() { trackingEnabled = true; }
  public void disableTracking() { trackingEnabled = false; }
  public boolean isTrackingEnabled() { return trackingEnabled; }

  // True when the current target bearing is within the turret's physical travel range.
  // False = target is in the ~28 deg blind spot; robot must rotate to bring it in range.
  public boolean isTurretReachable() { return aimGoal.targetReachable; }

  // True when the aim pipeline has computed a valid goal (pose is good and target is reachable).
  // Use this to confirm the turret is actually tracking before gating a shot.
  public boolean hasAimGoal() { return aimGoal.enable; }

  // RPS values last solved by the aim pipeline for the current robot distance.
  public double getAimGoalFrontRps() { return aimGoal.flywheelFrontRps; }
  public double getAimGoalBackRps()  { return aimGoal.flywheelBackRps; }

  // Returns true only when all conditions are satisfied for a safe shot
  public boolean isReadyToShoot() {
    if (!aimGoal.enable){
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "no aim goal");
      return false;
    }
    if (!aimGoal.targetReachable) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "target in deadzone");
      return false;
    }
    if (aimGoal.chassisSpeedMps > Constants.Turret.CHASSIS_SPEED_FIRE_THRESHOLD_MPS) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "chassis too fast");
      return false;
    }
    if (!isTurretOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "turret not on target");
      return false;
    }
    if (!isFlywheelOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "flywheel not up to speed");
      return false;
    }
    if (!isHoodOnTarget()) {
      SmartLogger.logReplay("Turret/ReadyToShoot", false);
      SmartLogger.logReplay("Turret/WhyNotReady", "hood not on target");
      return false;
    }
    SmartLogger.logReplay("Turret/ReadyToShoot", true);
    SmartLogger.logReplay("Turret/WhyNotReady", "");
    return true;
  }

  private boolean isTurretOnTarget() {
    if (!aimGoal.enable) {
      turretOnTargetLoops = 0;
      return true; // open loop — no target to check against
    }
    double error = Math.abs(aimGoal.turretRotations - inputs.turretAbsolutePositionRotations);
    if (error < Constants.Turret.TURRET_ON_TARGET_TOLERANCE_ROT) {
      turretOnTargetLoops++;
    } else {
      turretOnTargetLoops = 0;
    }
    return turretOnTargetLoops >= Constants.Turret.TURRET_ON_TARGET_SETTLE_LOOPS;
  }

  private boolean isFlywheelOnTarget() {
    double frontError = Math.abs(aimGoal.flywheelFrontRps - inputs.flywheelVelocityRpm / 60);
    double backError = Math.abs(aimGoal.flywheelFrontRps - inputs.flywheelBackVelocityRpm / 60);
    return (frontError < Constants.Turret.FLYWHEEL_ON_TARGET_TOLERANCE_RPS) && (backError < Constants.Turret.FLYWHEEL_ON_TARGET_TOLERANCE_RPS);
  }

  private boolean isHoodOnTarget() {
    if (!aimGoal.enable) return true;
    double error = Math.abs(aimGoal.hoodRotations - inputs.hoodMotorPositionRotations);
    return error < Constants.Turret.HOOD_ON_TARGET_TOLERANCE_ROT;
  }

  // Commands hood to a specific position. Requires hoodHomed. Clamps to soft limits.
  public void setHoodPositionTarget(double motorRotations) {
    if (!hoodHoming){
      setpoints.useHoodPosition = true;
      setpoints.hoodPositionMotorRotations = Math.max(0.0, Math.min(motorRotations, Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS));
    }
  }
  // Closed-loop velocity control — both motors independently in RPS (motor shaft rot/sec).
  // Measured 2026-03-07: 80% -> front=78.10 RPS, back=74.58 RPS. See Constants.Turret.MEASURED_*_RPS.
  public void setFlywheelFrontRps(double rps) {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = true;
    setpoints.flywheelFrontRps = rps;
  }
  public void setFlywheelBackRps(double rps) {
    setpoints.useIndependentFlywheel = true;
    setpoints.useFlywheelRps = true;
    setpoints.flywheelBackRps = rps;
  }
  public void stopFlywheel()  {
    setpoints.flywheelFrontRps = 0;
    setpoints.flywheelBackRps = 0;
  }

  // ---- Aim goal (closed loop) ----

  public void setAimGoal(TurretAimGoal goal) {
    aimGoal.turretRotations  = goal.turretRotations;
    aimGoal.hoodRotations    = goal.hoodRotations;
    aimGoal.flywheelFrontRps = goal.flywheelFrontRps;
    aimGoal.flywheelBackRps  = goal.flywheelBackRps;
    aimGoal.enable           = goal.enable;
    aimGoal.targetReachable  = goal.targetReachable;
    aimGoal.chassisSpeedMps  = goal.chassisSpeedMps;
  }

  public boolean updateAimFromProvider(TurretAimPipeline provider) {
    if (provider == null || !trackingEnabled) return false;
    // While stall give-up is active, block the pipeline from overwriting the snapped position.
    // The turret will resume tracking as soon as the chain frees and velocity recovers.
    if (stallGiveUpActive) return false;
    boolean valid = provider.update(providerGoal);
    // Always apply the goal — even when enable=false (deadzone), we need aimGoal.enable
    // and targetReachable to update so the turret holds position instead of chasing a stale target.
    manualPositionOverride = false;
    setAimGoal(providerGoal);
    return valid;
  }

  // Bypasses trackingEnabled — holds turret at forward (0 rot) under MotionMagic PID
  // with hood and flywheel set by distance. Used for Phase1Fallback and QuestNav emergency.
  // Safe to call every loop while either emergency flag is active.
  public void holdForwardUnderPID(double distanceMeters) {
    TurretShotProfile shot = TurretShotProfile.getForDistance(distanceMeters);
    providerGoal.turretRotations  = 0.0;
    providerGoal.hoodRotations    = shot.hoodRotations;
    providerGoal.flywheelFrontRps = shot.flywheelFrontRps;
    providerGoal.flywheelBackRps  = shot.flywheelBackRps;
    providerGoal.targetReachable  = true;
    providerGoal.enable           = true;
    manualPositionOverride = false;
    setAimGoal(providerGoal);
    SmartLogger.logReplay("Turret/HoldForward/DistanceM", distanceMeters);
    SmartLogger.logReplay("Turret/HoldForward/Active", true);
  }

  public void stopAll() {
    aimGoal.clear();
    setpoints.clear();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // If the motor rebooted mid-match (brownout), restore the encoder from our saved position.
    // The sticky fault fires for exactly one loop then is cleared by TurretIOCTRE.
    // Also patch inputs so the rest of this loop uses the correct position, not the reset zero.
    if (inputs.turretBootDuringEn) {
      io.restoreTurretEncoder(lastKnownPosition);
      inputs.turretAbsolutePositionRotations = lastKnownPosition;
      SmartLogger.logConsole("Turret motor reboot detected — encoder restored to " +
          String.format("%.3f", lastKnownPosition) + " rot", "Turret");
    }

    // Save position each loop after homing, but only if it changed meaningfully
    if (Math.abs(inputs.turretAbsolutePositionRotations - lastKnownPosition) > POSITION_SAVE_THRESHOLD) {
      lastKnownPosition = inputs.turretAbsolutePositionRotations;
    }

    // Complete hood homing — limit switch is primary, stall current is fallback.
    if (hoodHoming) {
      boolean limitHit = inputs.hoodLimitSwitchRaw;
      boolean stallHit = inputs.hoodMotorCurrentAmps >= Constants.Turret.HOOD_HOMING_STALL_CURRENT_AMPS;
      if (stallHit) {
        hoodHomingStallLoopCount++;
      } else {
        hoodHomingStallLoopCount = 0;
      }
      boolean stallConfirmed = hoodHomingStallLoopCount >= Constants.Turret.HOOD_HOMING_STALL_LOOP_THRESHOLD;
      if (limitHit || stallConfirmed) {
        hoodHoming = false;
        hoodHomingStallLoopCount = 0;
        io.zeroHoodEncoder();
        setpoints.hoodPositionMotorRotations = Constants.Turret.HOOD_HOME_BACKOFF_ROTATIONS;
        setpoints.useHoodPosition = true;
        setpoints.hoodPercent = 0.0;
        SmartLogger.logConsole(
            "Hood homed — encoder zeroed (" + (limitHit ? "limit switch" : "stall fallback") + ")", "Turret");
      }
    }

    // Hood soft limit: stop upward movement at the top of the travel range.
    // No physical hard stop at the top — this is the only protection against over-travel.
    if (!hoodHoming && inputs.hoodMotorPositionRotations >= Constants.Turret.HOOD_SOFT_LIMIT_TOP_ROTATIONS
        && setpoints.hoodPercent > 0.0) {
      setpoints.hoodPercent = 0.0;
    }

    // Turret soft limits: block open-loop percent commands that would drive past either end of travel.
    // Also clamps any active MotionMagic position target so it can never be commanded past the limits.
    // Chain-jam stall recovery: if the turret has been stuck (not moving, but not on target)
    // for TURRET_STALL_TIMEOUT_SECS, snap the MM target to the current position so it stops
    // fighting the jam. Clears automatically once the turret starts moving again.
    if (setpoints.useTurretPosition && edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
      double posErr = Math.abs(setpoints.turretPositionMotorRotations - inputs.turretAbsolutePositionRotations);
      boolean isStuck = Math.abs(inputs.turretVelocityRps) < Constants.Turret.TURRET_STALL_VELOCITY_THRESHOLD_RPS
          && posErr > Constants.Turret.TURRET_STALL_ERROR_THRESHOLD_ROT;

      if (isStuck) {
        stallTimerSecs += 0.02;
      } else {
        stallTimerSecs = 0.0;
        stallGiveUpActive = false;
      }

      if (stallTimerSecs >= Constants.Turret.TURRET_STALL_TIMEOUT_SECS && !stallGiveUpActive) {
        stallGiveUpActive = true;
        stallTimerSecs = Constants.Turret.TURRET_STALL_TIMEOUT_SECS;
        setpoints.turretPositionMotorRotations = inputs.turretAbsolutePositionRotations;
        SmartLogger.logConsole("Turret stall timeout — holding current position (chain jam?)", "Turret");
      }
      SmartLogger.logReplay("Turret/StallGiveUpActive", stallGiveUpActive);
      SmartLogger.logReplay("Turret/StallTimerSecs",    stallTimerSecs);
    } else {
      stallTimerSecs    = 0.0;
      stallGiveUpActive = false;
    }

    double turretPos = inputs.turretAbsolutePositionRotations;
    if (turretPos <= Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT && setpoints.turretPercent < 0.0) {
      setpoints.turretPercent = 0.0;
    }
    if (turretPos >= Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT && setpoints.turretPercent > 0.0) {
      setpoints.turretPercent = 0.0;
    }
    // Clamp the MotionMagic target so overshoots or stale setpoints can't command past the limits.
    if (setpoints.useTurretPosition) {
      setpoints.turretPositionMotorRotations = Math.max(
          Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
          Math.min(setpoints.turretPositionMotorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    }

    if (!manualPositionOverride) {
      setpointGenerator.update(aimGoal, setpoints);
    }

    if (setpoints.useIndependentFlywheel) {
      if (setpoints.useFlywheelRps) {
        io.setFlywheelFrontRps(setpoints.flywheelFrontRps);
        io.setFlywheelBackRps(setpoints.flywheelBackRps);
      } else {
        io.setFlywheelFrontPercent(setpoints.flywheelFrontPercent);
        io.setFlywheelBackPercent(setpoints.flywheelBackPercent);
      }
    } else {
      io.setFlywheelPercent(setpoints.flywheelPercent);
    }
    if (hoodHoming) {
      io.setHoodPercent(-Constants.Turret.HOOD_HOME_SPEED_PERCENT);
    } else if (setpoints.useHoodPosition) {
      io.setHoodPosition(setpoints.hoodPositionMotorRotations);
    } else {
      io.setHoodPercent(setpoints.hoodPercent);
    }

    robotState.setTurretFlywheelPercent(setpoints.flywheelPercent);
    robotState.setTurretHoodPercent(setpoints.hoodPercent);
    robotState.setTurretRotationPercent(setpoints.turretPercent);

    boolean active = Math.abs(setpoints.flywheelPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(setpoints.hoodPercent) > ACTIVE_PERCENT_THRESHOLD
        || Math.abs(setpoints.turretPercent) > ACTIVE_PERCENT_THRESHOLD;
    robotState.setTurretState(active ? RobotState.TurretState.ACTIVE : RobotState.TurretState.IDLE);

    robotState.setTurretHoodLimitSwitchRaw(inputs.hoodLimitSwitchRaw);
    robotState.setTurretHallCCWRaw(inputs.hallCCWRaw);
    robotState.setTurretHoodMotorPositionRotations(inputs.hoodMotorPositionRotations);
    robotState.setTurretRotationAbsolutePositionRotations(inputs.turretAbsolutePositionRotations);

    SmartLogger.logReplay("Turret/Phase", Constants.Turret.CURRENT_PHASE.toString());
    SmartLogger.logReplay("Turret/HoodHomed", !hoodHoming);
    SmartLogger.logReplay("Turret/HoodLimitSwitchPressed", inputs.hoodLimitSwitchRaw); // true = switch pressed (active-low after inversion)

    SmartLogger.logReplay("Turret/TargetMotorRot", setpoints.turretPositionMotorRotations);
    SmartLogger.logReplay("Turret/RotationMotorRot", inputs.turretAbsolutePositionRotations);
    SmartLogger.logReplay("Turret/VelocityRps", inputs.turretVelocityRps);
    SmartLogger.logReplay("Turret/CurrentAmps", inputs.turretMotorCurrentAmps);

    // Hood PID tuning signals — plot HoodTargetRot vs HoodActualRot to tune kP/kV/kS
    SmartLogger.logReplay("Turret/HoodTargetRot", setpoints.hoodPositionMotorRotations);
    SmartLogger.logReplay("Turret/HoodActualRot", inputs.hoodMotorPositionRotations);
    SmartLogger.logReplay("Turret/HoodErrorRot",  setpoints.hoodPositionMotorRotations - inputs.hoodMotorPositionRotations);
    SmartLogger.logReplay("Turret/HoodCurrentAmps", inputs.hoodMotorCurrentAmps);

    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/RotationDeg", inputs.turretAbsolutePositionRotations * 360.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/HoodRotations", inputs.hoodMotorPositionRotations);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HoodHomed", !hoodHoming);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/HallCCW", inputs.hallCCWRaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontRpm", inputs.flywheelVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackRpm", inputs.flywheelBackVelocityRpm);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontTargetRps", setpoints.flywheelFrontRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackTargetRps", setpoints.flywheelBackRps);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelFrontActualRps", inputs.flywheelVelocityRpm / 60.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelBackActualRps", inputs.flywheelBackVelocityRpm / 60.0);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Turret/FlywheelSetpointPct", setpoints.flywheelFrontPercent);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
        "Turret/ReadyToShoot", isReadyToShoot());
    SmartLogger.logReplay("Turret/TargetReachable", aimGoal.targetReachable);
    robotState.setDeadzoneSuppressed(!aimGoal.targetReachable);
  }

  // Sets a closed-loop position target in motor rotations (0 = CCW hard stop).
  // Clamped to safe travel range. Requires homing to be complete.
  // Bypasses the aim pipeline — writes directly to setpoints so TURRET_FORWARD_MOTOR_ROT is not added.
  public void setTurretPositionTarget(double motorRotations) {
    double clamped = Math.max(Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
                              Math.min(motorRotations, Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT));
    manualPositionOverride = true;
    setpoints.turretPositionMotorRotations = clamped;
    setpoints.useTurretPosition = true;
  }
}