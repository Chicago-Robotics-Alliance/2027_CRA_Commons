// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import static frc.lib.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;

/** Add your docs here. */
public class MotorIOSpark extends MotorIO {
  protected final SparkBase main;
  protected final SparkBase[] followers;
  protected final SparkClosedLoopController mainController;
  protected SparkBaseConfig config;
  protected SparkBaseConfig followerConfig;
  private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
  private ThreadPoolExecutor threadPoolExecutor =
      new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);
  private final Debouncer sparkConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public void applyConfig(SparkBase spark, SparkBaseConfig config) {
    config.encoder.positionConversionFactor(1);
    config.encoder.velocityConversionFactor(1);
    config.absoluteEncoder.positionConversionFactor(1);
    config.absoluteEncoder.velocityConversionFactor(1);
    threadPoolExecutor.submit(
        () -> {
          for (int i = 0; i < 5; i++) {
            REVLibError error =
                spark.configure(
                    config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (error == REVLibError.kOk) {
              break;
            } else if (i == 4) {
              System.err.println(
                  "Failed to apply config to Spark"
                      + spark.getDeviceId()
                      + " after 5 attempts: "
                      + error.toString());
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

  protected void updateMotorInputs(MotorIOInputs inputs, SparkBase spark) {
    sparkStickyFault = false;
    ifOk(
        spark,
        new DoubleSupplier[] {
          spark.getEncoder()::getPosition,
          spark.getEncoder()::getVelocity,
          spark::getOutputCurrent,
          spark::getAppliedOutput,
          spark::getBusVoltage,
          spark::getMotorTemperature,
        },
        (values) -> {
          inputs.position = Units.Rotations.of(values[0]);
          inputs.velocity = Units.RotationsPerSecond.of(values[1] / 60.0); // Convert RPM to RPS
          inputs.statorCurrent = Units.Amps.of(values[2]);
          inputs.supplyCurrent =
              Units.Amps.of(
                  values[2]
                      * values[3]); // Approximation based on applied output and output current
          inputs.motorVoltage = Units.Volts.of(values[4]);
          inputs.temperature = Units.Celsius.of(values[5]);
        });
    inputs.connected = sparkConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setNeutralSetpoint() {
    main.stopMotor();
  }

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {
    main.setVoltage(voltage.in(Units.Volts));
  }

  @Override
  protected void setDutyCycleSetpoint(Dimensionless percent) {
    main.set(percent.in(Units.Percent) / 100.0);
  }

  @Override
  protected void setPositionSetpoint(Angle position) {
    mainController.setSetpoint(
        position.in(Units.Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  protected void setSmartPositionSetpoint(Angle position) {
    mainController.setSetpoint(
        position.in(Units.Rotations), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  @Override
  protected void setVelocitySetpoint(AngularVelocity velocity) {
    mainController.setSetpoint(
        velocity.in(Units.RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot2);
  }

  @Override
  protected void setSmartVelocitySetpoint(AngularVelocity velocity) {
    mainController.setSetpoint(
        velocity.in(Units.RPM), ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot3);
  }

  @Override
  public void setCurrentPosition(Angle position) {
    threadPoolExecutor.submit(
        () -> {
          main.getEncoder().setPosition(position.in(Units.Rotations));
        });
  }

  @Override
  public void zeroSensors() {
    setCurrentPosition(Units.Rotations.of(0.0));
  }

  @Override
  public void setNeutralBrake(boolean shouldBrake) {
    UnaryOperator<SparkBaseConfig> configChanger =
        (config) -> {
          config.idleMode(
              shouldBrake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
          return config;
        };

    changeMainConfig(configChanger);
    changeFollowerConfig(configChanger);
  }

  @Override
  public void useSoftLimits(boolean enable) {
    UnaryOperator<SparkBaseConfig> configChanger =
        (config) -> {
          config.softLimit.forwardSoftLimitEnabled(enable);
          config.softLimit.reverseSoftLimitEnabled(enable);
          return config;
        };

    changeMainConfig(configChanger);
  }

  /**
   * Applies a SparkBaseConfig to the main motor.
   *
   * @param configuration Configuration to apply.
   */
  public void setMainConfig(SparkBaseConfig configuration) {
    config = configuration;
    applyConfig(main, config);
  }

  /**
   * Changes the currently applied main SparkBaseConfig and applies the new configuration to the
   * main motor.
   *
   * @param configChanger Mutating operation to apply on the current configuration.
   */
  public void changeMainConfig(UnaryOperator<SparkBaseConfig> configChanger) {
    setMainConfig(configChanger.apply(config));
  }

  /**
   * Applies a SparkBaseConfig to all follower motors.
   *
   * @param configuration Configuration to apply.
   */
  public void setFollowerConfig(SparkBaseConfig configuration) {
    followerConfig = configuration;
    for (SparkBase spark : followers) {
      applyConfig(spark, followerConfig);
    }
  }

  /**
   * Changes the currently applied follower SparkBaseConfig and applies the new configuration to all
   * follower motors.
   *
   * @param configChanger Mutating operation to apply on the current configuration.
   */
  public void changeFollowerConfig(UnaryOperator<SparkBaseConfig> configChanger) {
    setFollowerConfig(configChanger.apply(followerConfig));
  }

  /**
   * Creates a MotorIOSpark from a provided configuration.
   *
   * @param config Configuration to create MotorIOSpark from.
   */
  public MotorIOSpark(MotorIOSparkConfig config) {
    super(config.unit, config.time, config.followerIDs.length);
    main =
        config.useFlex
            ? new SparkFlex(config.mainID, MotorType.kBrushless)
            : new SparkMax(config.mainID, MotorType.kBrushless);
    mainController = main.getClosedLoopController();
    setMainConfig(config.mainConfig);

    followers = new SparkBase[config.followerIDs.length];
    for (int i = 0; i < config.followerIDs.length; i++) {
      followers[i] =
          config.useFlex
              ? new SparkFlex(config.followerIDs[i], MotorType.kBrushless)
              : new SparkMax(config.followerIDs[i], MotorType.kBrushless);
      config.followerConfig.follow(config.mainID, config.followerOpposeMain[i]);
    }

    setFollowerConfig(config.followerConfig);
  }

  /**
   * Configuration for a MotorIOSpark. Position is on slot 0, MAXMotion position is on slot 1,
   * Velocity is on slot 2, and MAXMotion velocity is on slot 3.
   */
  public static class MotorIOSparkConfig {
    public AngleUnit unit = Units.Rotations;
    public TimeUnit time = Units.Seconds;
    public int mainID = -1;
    public boolean useFlex = false;
    public SparkBaseConfig mainConfig;
    public int[] followerIDs = new int[0];
    public SparkBaseConfig followerConfig;
    public boolean[] followerOpposeMain = new boolean[0];
  }
}
