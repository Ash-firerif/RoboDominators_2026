package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants;
import frc.robot.RobotState;

public class LaunchCalculator {
  private static LaunchCalculator instance;
  private TurretTargetSelector target = new TurretTargetSelector(RobotState.getInstance());

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      double hoodRots,
      double frontFlywheelSpeed,
      double backFlywheelSpeed,
      double turretRots,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  private LaunchingParameters latestParameters = null;

  public LaunchingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }
    Pose2d estimatedPose = RobotState.getInstance().getRobotPose();
    ChassisSpeeds robotVelocity = RobotState.getInstance().getRobotVelocity();

    double dx = target.get().getX() - estimatedPose.getX();
    double dy = target.get().getY() - estimatedPose.getY();
    double distance = Math.hypot(dx, dy);

    double offX = Constants.Turret.TURRET_PIVOT_OFFSET_X_METERS;
    double offY = Constants.Turret.TURRET_PIVOT_OFFSET_Y_METERS;

    double heading = estimatedPose.getRotation().getRadians();
    double pivotFieldDx = offX * Math.cos(heading) - offY * Math.sin(heading);
    double pivotFieldDy = offX * Math.sin(heading) + offY * Math.cos(heading);

    // omega is stored as abs — recover signed omega from chassis speeds field directly via inputs
    double signedOmega = robotVelocity.omegaRadiansPerSecond; // positive = CCW

    double tof = TurretShotProfile.getForDistance(distance).timeOfFlightSeconds;
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;
    // Tangential velocity: perpendicular to pivot offset vector, CCW positive
    double vtx = -signedOmega * pivotFieldDy;
    double vty =  signedOmega * pivotFieldDx;
    vx += vtx;
    vy += vty;
    
    dx = dx - vx * tof;
    dy = dy - vy * tof;
    double distanceLead = Math.hypot(dx, dy);

    // Normalize the turret-relative angle to [-π, π] so rawTurretRotations stays in [-0.5, +0.5].
    // atan2(sin,cos) normalization handles the ±π wrap at ~180° heading (Red alliance).
    // Negate because the turret motor's positive direction is CW, opposite to CCW field-frame convention.
    double relativeRad = Math.atan2(dy, dx) - heading;
    double turretRelativeRad = Math.atan2(Math.sin(relativeRad), Math.cos(relativeRad));
    double motorTargetRaw = MathUtil.clamp(-(turretRelativeRad / (2.0 * Math.PI)) *
        Constants.Turret.TURRET_GEAR_RATIO + Constants.Turret.TURRET_FORWARD_MOTOR_ROT,
        Constants.Turret.TURRET_SOFT_LIMIT_LEFT_MOTOR_ROT,
        Constants.Turret.TURRET_SOFT_LIMIT_RIGHT_MOTOR_ROT);
    TurretShotProfile shot = TurretShotProfile.getForDistance(distanceLead);

    latestParameters = new LaunchingParameters(
      true,
      shot.hoodRotations,
      shot.flywheelFrontRps,
      shot.flywheelBackRps,
      motorTargetRaw,
      distanceLead,
      distance,
      tof,
      false);

    return latestParameters;
  }
}
