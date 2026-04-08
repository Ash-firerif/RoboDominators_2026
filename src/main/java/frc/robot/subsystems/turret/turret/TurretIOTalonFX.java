package frc.robot.subsystems.turret.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class TurretIOTalonFX implements TurretIO{

  private final TalonFX motor;
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagic  = new MotionMagicVoltage(0.0).withSlot(0);

  public TurretIOTalonFX(){
    motor = new TalonFX(Constants.Turret.TURRET_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Constants.Turret.TURRET_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    config.CurrentLimits.StatorCurrentLimit       = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = Constants.Turret.TURRET_KS;
    config.Slot0.kV = Constants.Turret.TURRET_KV;
    config.Slot0.kP = Constants.Turret.TURRET_KP;

    MotionMagicConfigs MM = new MotionMagicConfigs();
    MM.MotionMagicCruiseVelocity = Constants.Turret.TURRET_CRUISE_VELOCITY_RPS;
    MM.MotionMagicAcceleration   = Constants.Turret.TURRET_ACCELERATION_RPS2;
    config.MotionMagic = MM;

    motor.getConfigurator().apply(config);
  }
  
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRots = motor.getPosition().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode){
      case OPEN_LOOP -> {
        motor.setControl(voltageOut.withOutput(outputs.appliedVolts));
      }
      case CLOSED_LOOP -> {
        motor.setControl(motionMagic.withPosition(outputs.positionRots));
      }
      case BRAKE -> {
        motor.setControl(voltageOut.withOutput(0));
      }
    }
  }
}