package frc.robot.Util;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Log.LogManager;

/** Add your docs here. */
public class TalonMotor extends TalonFX {
  TalonConfig config;
  String name;
  TalonFXConfiguration cfg;

  public TalonMotor(TalonConfig config) {
    super(config.id, config.canbus);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
  }

  private void configMotor() {
    cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentThreshold = config.maxCurrentTriggerTime;
    cfg.CurrentLimits.SupplyTimeThreshold = config.maxCurrentTriggerTime;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

    cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
    cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;

    cfg.Slot0.kP = config.pid.kp;
    cfg.Slot0.kI = config.pid.ki;
    cfg.Slot0.kD = config.pid.kd;
    cfg.Slot0.kS = config.pid.ks; // add 0.5 Volts
    cfg.Slot0.kV = config.pid.kv;
    cfg.Slot0.kA = config.pid.ka;

    cfg.Voltage.PeakForwardVoltage = config.maxVolt;
    cfg.Voltage.PeakReverseVoltage = config.minVolt;

    cfg.Feedback.SensorToMechanismRatio = config.motorRatio;

    getConfigurator().apply(cfg);
  }

  public void setBrake(boolean brake) {
    this.getConfigurator().refresh(cfg.MotorOutput);
    cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    getConfigurator().apply(cfg.MotorOutput);
  }

  private void addLog() {
    LogManager.addEntry(name + "/position", this::getPosition, true);
    LogManager.addEntry(name + "/Velocity", this::getVelocity, true);
    LogManager.addEntry(name + "/Acceleration", this::getAcceleration, true);
    LogManager.addEntry(name + "/Voltage", this::getMotorVoltage, true);
    LogManager.addEntry(name + "/Current", this::getStatorCurrent, true);
    LogManager.addEntry(name + "/CloseLoopError", this::getClosedLoopError, true);
    LogManager.addEntry(name + "/CloseLoopOutput", this::getClosedLoopOutput, true);
    LogManager.addEntry(name + "/CloseLoopP", this::getClosedLoopProportionalOutput, true);
    LogManager.addEntry(name + "/CloseLoopI", this::getClosedLoopIntegratedOutput, true);
    LogManager.addEntry(name + "/CloseLoopD", this::getClosedLoopDerivativeOutput, true);
    LogManager.addEntry(name + "/CloseLoopFF", this::getClosedLoopFeedForward, true);
    LogManager.addEntry(name + "/CloseLoopSP", this::getClosedLoopReference, true);
  }

}
