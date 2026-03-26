package frc.robot.subsystems.turret.hood;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO{

  private final TalonFX motor;
  private final DigitalInput limitSwitch;
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagic  = new MotionMagicVoltage(0.0).withSlot(0);

  public HoodIOTalonFX(){
    motor = new TalonFX(Constants.Turret.HOOD_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = Constants.Turret.HOOD_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    config.CurrentLimits.StatorCurrentLimit       = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = Constants.Turret.HOOD_KS;
    config.Slot0.kV = Constants.Turret.HOOD_KV;
    config.Slot0.kP = Constants.Turret.HOOD_KP;

    MotionMagicConfigs MM = new MotionMagicConfigs();
    MM.MotionMagicCruiseVelocity = Constants.Turret.HOOD_CRUISE_VELOCITY_RPS;
    MM.MotionMagicAcceleration   = Constants.Turret.HOOD_ACCELERATION_RPS2;
    config.MotionMagic = MM;

    motor.getConfigurator().apply(config);
    limitSwitch = new DigitalInput(Constants.Turret.HOOD_LIMIT_SWITCH_DIO);

  }
  
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.limitSwitch = limitSwitch.get();
    inputs.positionRots = motor.getPosition().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getTorqueCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
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