package frc.robot.subsystems.turret.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO{

  private final TalonFX frontMotor;
  private final TalonFX backMotor;
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final VelocityVoltage flywheelFrontVelocity = new VelocityVoltage(0.0).withSlot(0);
  private final VelocityVoltage flywheelBackVelocity  = new VelocityVoltage(0.0).withSlot(0);

  public FlywheelIOTalonFX(){
    frontMotor = new TalonFX(Constants.Turret.FLYWHEEL_FRONT_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));
    backMotor = new TalonFX(Constants.Turret.FLYWHEEL_BACK_MOTOR_ID, new CANBus(Constants.Swerve.CAN_BUS_NAME));

    // Flywheel motors: coast mode, separate inversion flags (back motor runs opposite to front)
    TalonFXConfiguration frontConfig = new TalonFXConfiguration();
    frontConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    frontConfig.CurrentLimits.StatorCurrentLimit       = 40.0;
    frontConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Slot 0: velocity feedforward + P gain from SysId 2026-03-07

    frontConfig.Slot0.kS = Constants.Turret.FLYWHEEL_FRONT_KS;
    frontConfig.Slot0.kV = Constants.Turret.FLYWHEEL_FRONT_KV;
    frontConfig.Slot0.kA = Constants.Turret.FLYWHEEL_FRONT_KA;
    frontConfig.Slot0.kP = Constants.Turret.FLYWHEEL_FRONT_KP;

    frontConfig.MotorOutput.Inverted = Constants.Turret.FLYWHEEL_FRONT_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    frontMotor.getConfigurator().apply(frontConfig);

    TalonFXConfiguration backConfig = new TalonFXConfiguration();

    backConfig.Slot0.kS = Constants.Turret.FLYWHEEL_BACK_KS;
    backConfig.Slot0.kV = Constants.Turret.FLYWHEEL_BACK_KV;
    backConfig.Slot0.kA = Constants.Turret.FLYWHEEL_BACK_KA;
    backConfig.Slot0.kP = Constants.Turret.FLYWHEEL_BACK_KP;

    backConfig.MotorOutput.Inverted = Constants.Turret.FLYWHEEL_BACK_MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    backMotor.getConfigurator().apply(backConfig);
  }
  
  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.frontVelocityRPS = frontMotor.getVelocity().getValueAsDouble();
    inputs.frontAppliedVolts = frontMotor.getMotorVoltage().getValueAsDouble();
    inputs.frontCurrentAmps = frontMotor.getTorqueCurrent().getValueAsDouble();
    inputs.frontCurrentAmps = frontMotor.getDeviceTemp().getValueAsDouble();

    inputs.backVelocityRPS = backMotor.getVelocity().getValueAsDouble();
    inputs.backAppliedVolts = backMotor.getMotorVoltage().getValueAsDouble();
    inputs.backCurrentAmps = backMotor.getTorqueCurrent().getValueAsDouble();
    inputs.backCurrentAmps = backMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode){
      case COAST -> {
        frontMotor.setControl(voltageOut.withOutput(outputs.frontAppliedVolts));
        backMotor.setControl(voltageOut.withOutput(outputs.backAppliedVolts));
      }
      case VELOCITY -> {
        frontMotor.setControl(flywheelFrontVelocity.withVelocity(outputs.frontVelocityRPS));
        backMotor.setControl(flywheelBackVelocity.withVelocity(outputs.backVelocityRPS));
      }
    }
  }
}