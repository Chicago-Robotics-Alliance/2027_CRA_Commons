// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.UnaryOperator;

/** Class used to control a main TalonFX and any number of followers for a real mechanism. */
public class MotorIOTalon extends MotorIO {
  protected final TalonFX main;
  protected final TalonFX[] followers;
  protected TalonFXConfiguration config;
  protected TalonFXConfiguration followerConfig;
  private final ControlRequestGetter requestGetter;
  private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);

  public void applyConfig(TalonFX fx, TalonFXConfiguration config) {
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            StatusCode result = fx.getConfigurator().apply(config);
            if (result.isOK()) {
              break;
            }
          }
        });
  }

  @Override
  public void updateInputs() {
    updateMotorInputs(inputs, main);

    for (int i = 0; i < followers.length; i++) {
      updateMotorInputs(followerInputs[i], followers[i]);
    }
  }

  /**
   * Updates one Inputs from readings of a TalonFX motor
   *
   * @param inputs Inputs to update from reading.
   * @param motor Motor to read from.
   */
  protected void updateMotorInputs(MotorIOInputs inputs, TalonFX motor) {
    inputs.position = motor.getPosition().getValue();
    inputs.velocity = motor.getVelocity().getValue();
    inputs.statorCurrent = motor.getStatorCurrent().getValue();
    inputs.supplyCurrent = motor.getSupplyCurrent().getValue();
    inputs.motorVoltage = motor.getMotorVoltage().getValue();
    inputs.temperature = motor.getDeviceTemp().getValue();
    inputs.connected = motor.isConnected();
  }

  private void setControl(ControlRequest request) {
    main.setControl(request);
  }

  @Override
  public void setNeutralSetpoint() {
    setControl(new NeutralOut());
  }

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {
    setControl(requestGetter.getVoltageRequest(voltage));
  }

  @Override
  protected void setDutyCycleSetpoint(Dimensionless percent) {
    setControl(requestGetter.getDutyCycleRequest(percent));
  }

  @Override
  protected void setSmartPositionSetpoint(Angle mechanismPosition) {
    setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
  }

  @Override
  protected void setSmartVelocitySetpoint(AngularVelocity mechanismVelocity) {
    setControl(requestGetter.getVelocityRequest(mechanismVelocity));
  }

  @Override
  protected void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
    setControl(requestGetter.getVelocityRequest(mechanismVelocity));
  }

  @Override
  protected void setPositionSetpoint(Angle mechanismPosition) {
    setControl(requestGetter.getPositionRequest(mechanismPosition));
  }

  @Override
  public void setCurrentPosition(Angle mechanismPosition) {
    threadPoolExecutor.submit(
        () -> {
          main.setPosition(mechanismPosition);
        });
  }

  @Override
  public void zeroSensors() {
    setCurrentPosition(Units.Rotations.of(0.0));
  }

  private void setNeutralMode(TalonFX fx, NeutralModeValue neutralMode) {
    SmartDashboard.putNumber("TALON FX NEUTRAL MODE SET!!", Timer.getFPGATimestamp());
    threadPoolExecutor.submit(
        () -> {
          fx.setNeutralMode(neutralMode);
        });
  }

  @Override
  public void setNeutralBrake(boolean wantsBrake) {
    NeutralModeValue neutralMode = wantsBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.NeutralMode = neutralMode;
    setNeutralMode(main, neutralMode);
    for (TalonFX talon : followers) {
      setNeutralMode(talon, neutralMode);
    }
  }

  @Override
  public void useSoftLimits(boolean enable) {
    UnaryOperator<TalonFXConfiguration> configChanger =
        (config) -> {
          config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
          config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
          return config;
        };

    changeMainConfig(configChanger);
  }

  /**
   * Applies a TalonFXConfiguration to the main motor.
   *
   * @param configuration Configuration to apply.
   */
  public void setMainConfig(TalonFXConfiguration configuration) {
    config = configuration;
    applyConfig(main, config);
  }

  /**
   * Changes the currently applied main TalonFXConfiguration and applies the new configuration to
   * the main motor.
   *
   * @param configChanger Mutating operation to apply on the current configuration.
   */
  public void changeMainConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
    setMainConfig(configChanger.apply(config));
  }

  /**
   * Applies a TalonFXConfiguration to all follower motors.
   *
   * @param configuration Configuration to apply.
   */
  public void setFollowerConfig(TalonFXConfiguration configuration) {
    followerConfig = configuration;
    for (TalonFX talon : followers) {
      applyConfig(talon, followerConfig);
    }
  }

  /**
   * Changes the currently applied follower TalonFXConfiguration and applies the new configuration
   * to all follower motors.
   *
   * @param configChanger Mutating operation to apply on the current configuration.
   */
  public void changeFollowerConfig(UnaryOperator<TalonFXConfiguration> configChanger) {
    setFollowerConfig(configChanger.apply(followerConfig));
  }

  /**
   * Creates a MotorIOTalonFX from a provided configuration.
   *
   * @param config Configuration to create MotorIOTalonFX from.
   */
  public MotorIOTalon(MotorIOTalonConfig config) {
    super(config.unit, config.time, config.followerIDs.length);
    requestGetter = config.requestGetter;
    main = new TalonFX(config.mainID, new CANBus(config.mainBus));
    setMainConfig(config.mainConfig);

    followers = new TalonFX[config.followerIDs.length];
    for (int i = 0; i < config.followerIDs.length; i++) {
      followers[i] = new TalonFX(config.followerIDs[i], new CANBus(config.followerBuses[i]));
      followers[i].setControl(
          new Follower(
              config.mainID,
              config.followerOpposeMain[i]
                  ? MotorAlignmentValue.Opposed
                  : MotorAlignmentValue.Aligned));
    }

    setFollowerConfig(followerConfig);
  }

  /**
   * Configuration for a MotorIOTalon. Position PID is on slot 0, motion magic control is on slot 1,
   * and velocity is on slot 2.
   */
  public static class MotorIOTalonConfig {
    public AngleUnit unit = Units.Rotations;
    public TimeUnit time = Units.Seconds;
    public int mainID = -1;
    public String mainBus = "ASSIGN_BUS";
    public TalonFXConfiguration mainConfig = new TalonFXConfiguration();
    public int[] followerIDs = new int[0];
    public String[] followerBuses = new String[0];
    public TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    public boolean[] followerOpposeMain = new boolean[0];
    public ControlRequestGetter requestGetter = new ControlRequestGetter();
  }

  public static class ControlRequestGetter {
    public ControlRequest getVoltageRequest(Voltage voltage) {
      return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
    }

    public ControlRequest getDutyCycleRequest(Dimensionless percent) {
      return new DutyCycleOut(percent.in(Units.Percent));
    }

    public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
      return new MotionMagicExpoVoltage(mechanismPosition).withSlot(1).withEnableFOC(true);
    }

    public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
      return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(2);
    }

    public ControlRequest getPositionRequest(Angle mechanismPosition) {
      return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(0);
    }
  }
}
