// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Abstract base class used to control a main motor and any number of followers as part of a
 * mechanism.
 */
public abstract class MotorIO {
  public final AngleUnit unitType;
  public final TimeUnit time;
  protected final MotorIOInputs inputs;
  protected final MotorIOInputs[] followerInputs;
  private Setpoint setpoint = Setpoint.withNeutralSetpoint();
  private boolean enabled = true;

  /** Updates MotorIO's inputs with values from the motor. */
  public abstract void updateInputs();

  /**
   * Sets the mechanism's current location as a given position.
   *
   * @param position The position to set the mechanism to.
   */
  public abstract void setCurrentPosition(Angle position);

  /** Set's the mechanisms current location as zero. */
  public abstract void zeroSensors();

  /**
   * Sets the motor to brake or coast mode when idle.
   *
   * @param shouldBrake Whether to brake or coast. True is brake, false is coast.
   */
  public abstract void setNeutralBrake(boolean shouldBrake);

  /** Sets the motor to use or ignore soft limits. */
  public abstract void useSoftLimits(boolean enable);

  /**
   * Sets the motor to be idle. Should not be called directly, only applied through {@link
   * Setpoint}.
   */
  protected abstract void setNeutralSetpoint();

  /**
   * Sets the motor to run at a given voltage. Should not be called directly, only applied through
   * {@link Setpoint}.
   *
   * @param voltage The voltage to set the motor to.
   */
  protected abstract void setVoltageSetpoint(Voltage voltage);

  /**
   * Sets the motor to go to a given velocity. Should not be called directly, only applied through
   * {@link Setpoint}.
   *
   * @param velocity The velocity to set the motor to.
   */
  protected abstract void setVelocitySetpoint(AngularVelocity velocity);

  /**
   * Sets the motor to go to a given position. Should not be called directly, only applied through
   * {@link Setpoint}.
   *
   * @param position The position to set the motor to.
   */
  protected abstract void setPositionSetpoint(Angle position);

  /**
   * Sets the motor to go to a given velocity using either MotionMagic or MAXMotion. Should not be
   * called directly, only applied through {@link Setpoint}.
   *
   * @param velocity The velocity to set the motor to.
   */
  protected abstract void setSmartVelocitySetpoint(AngularVelocity velocity);

  /**
   * Sets the motor to go to a given position using either MotionMagic or MAXMotion. Should not be
   * called directly, only applied through {@link Setpoint}.
   *
   * @param position The position to set the motor to.
   */
  protected abstract void setSmartPositionSetpoint(Angle position);

  /**
   * Sets the motor to run at a percentage of its max voltage. Should not be called directly, only
   * applied through {@link Setpoint}.
   *
   * @param percent The percentage to set the motor to.
   */
  protected abstract void setDutyCycleSetpoint(Dimensionless percent);

  /**
   * Applies a setpoint to the MotorIO.
   *
   * @param desiredSetpoint The setpoint to apply.
   */
  public final void applySetpoint(Setpoint desiredSetpoint) {
    setpoint = desiredSetpoint;
    if (enabled) {
      desiredSetpoint.apply(this);
    }
  }

  /**
   * Enables this MotorIO. Immediately applies the last Setpoint, including Setpoints set while
   * disabled. MotorIO is enabled by default.
   */
  public final void enable() {
    enabled = true;
    setpoint.apply(this);
  }

  /**
   * Disables this MotorIO. Setpoints can be set while disabled but will not be applied until
   * re-enabled.
   */
  public final void disable() {
    enabled = false;
    Setpoint.withNeutralSetpoint().apply(this);
  }

  /**
   * Gets whether this MotorIO is enabled.
   *
   * @return True if enabled, false if disabled.
   */
  public boolean getEnabled() {
    return enabled;
  }

  /**
   * Creates a new MotorIO with no followers.
   *
   * @param unit Units to measure.
   * @param time Time units to measure.
   */
  protected MotorIO(AngleUnit unit, TimeUnit time) {
    this(unit, time, 0);
  }

  /**
   * Creates a new MotorIO with a given number of followers.
   *
   * @param unit Units to measure.
   * @param time Time units to measure.
   * @param numFollowers Number of followers motors.
   */
  protected MotorIO(AngleUnit unit, TimeUnit time, int numFollowers) {
    this.unitType = unit;
    this.time = time;
    inputs = new MotorIOInputsAutoLogged();
    followerInputs = new MotorIOInputs[numFollowers];

    for (int i = 0; i < numFollowers; i++) {
      followerInputs[i] = new MotorIOInputsAutoLogged();
    }
  }

  /**
   * Processes logging for this MotorIO and its followers. This method should be called periodically
   * by the subsystem controlling the motor.
   *
   * @param name The name used for logging purposes. (Ideally, pass the subsystem name for
   *     simplicity.)
   */
  public void processLogging(String name) {
    Logger.processInputs(name, (LoggableInputs) inputs);
    for (int i = 0; i < followerInputs.length; i++) {
      Logger.processInputs(name + "/Follower" + i, (LoggableInputs) followerInputs[i]);
    }
  }

  /**
   * Gets the last read velocity of the main motor.
   *
   * @return Velocity of mechanism.
   */
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /**
   * Gets the last read position of the main motor.
   *
   * @return Position of mechanism.
   */
  public Angle getPosition() {
    return inputs.position;
  }

  /**
   * Gets the last read stator current of the main motor.
   *
   * @return Stator current.
   */
  public Current getStatorCurrent() {
    return inputs.statorCurrent;
  }

  /**
   * Gets the last read supply current of the main motor.
   *
   * @return Supply current.
   */
  public Current getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  /**
   * Gets the last read output voltage of the main motor.
   *
   * @return Output voltage.
   */
  public Voltage getMotorVoltage() {
    return inputs.motorVoltage;
  }

  /**
   * Gets the last applied Setpoint of the MotorIO.
   *
   * @return Last applied Setpoint.
   */
  public Setpoint getSetpoint() {
    return setpoint;
  }

  /**
   * Gets the last applied Setpoint of the MotorIO in units of the MotorIO.
   *
   * @return Setpoint in mechanism units.
   */
  public double getSetpointDoubleInUnits() {
    Setpoint currentSetpoint = getSetpoint();
    switch (currentSetpoint.mode) {
      case POSITION, SMART_POS:
        AngleUnit positionUnit = unitType;
        return positionUnit.ofBaseUnits(currentSetpoint.baseUnits).in(positionUnit);
      case VELOCITY, SMART_VEL:
        AngularVelocityUnit velocityUnit = unitType.per(time);
        return velocityUnit.ofBaseUnits(currentSetpoint.baseUnits).in(velocityUnit);
      case VOLTAGE:
        VoltageUnit voltageUnit = Units.Volts;
        return voltageUnit.ofBaseUnits(currentSetpoint.baseUnits).in(voltageUnit);
      case DUTY_CYCLE:
        DimensionlessUnit percentUnit = Units.Percent;
        return percentUnit.ofBaseUnits(currentSetpoint.baseUnits).in(percentUnit);
      case IDLE:
      default:
        return currentSetpoint.baseUnits;
    }
  }

  /**
   * Class to store readings from a motor. Readings should be stored relative to the mechanism the
   * motor is controlling.
   */
  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public AngularVelocity velocity = BaseUnits.AngleUnit.of(0.0).per(Units.Second);
    public Angle position = BaseUnits.AngleUnit.of(0.0);
    public Current statorCurrent = BaseUnits.CurrentUnit.of(0.0);
    public Current supplyCurrent = BaseUnits.CurrentUnit.of(0.0);
    public Voltage motorVoltage = BaseUnits.VoltageUnit.of(0.0);
    public Temperature temperature = BaseUnits.TemperatureUnit.of(0.0);
  }

  /** Represents different control modes for a MotorIO. */
  public enum Mode {
    IDLE,
    VOLTAGE,
    VELOCITY,
    POSITION,
    SMART_VEL, // MotionMagic or MAXMotion velocity control
    SMART_POS, // MotionMagic or MAXMotion position control
    DUTY_CYCLE;

    /**
     * Gets whether this mode is a position control mode.
     *
     * @return True if this mode is a position control mode, false otherwise.
     */
    public boolean isPositionControl() {
      return switch (this) {
        case POSITION, SMART_POS -> true;
        default -> false;
      };
    }

    /**
     * Gets whether this mode is a velocity control mode.
     *
     * @return True if this mode is a velocity control mode, false otherwise.
     */
    public boolean isVelocityControl() {
      return switch (this) {
        case VELOCITY, SMART_VEL -> true;
        default -> false;
      };
    }

    /**
     * Gets whether this mode is a neutral control mode (i.e., idle).
     *
     * @return True if this mode is a neutral control mode, false otherwise.
     */
    public boolean isNeutralControl() {
      return switch (this) {
        case IDLE -> true;
        default -> false;
      };
    }

    /**
     * Gets whether this mode is a voltage control mode. Voltage and Duty Cycle control count as
     * voltage.
     *
     * @return True if this mode is a voltage control mode, false otherwise.
     */
    public boolean isVoltageControl() {
      return switch (this) {
        case VOLTAGE, DUTY_CYCLE -> true;
        default -> false;
      };
    }
  }

  /** Setpoint for a MotorIO. */
  public static class Setpoint {
    private final UnaryOperator<MotorIO> applier;
    public final Mode mode;
    public final double baseUnits;

    /**
     * Creates a new Setpoint with a given applier, mode, and base units.
     *
     * @param applier The function that applies the setpoint to a MotorIO.
     * @param mode The control mode of the setpoint.
     * @param baseUnits The Setpoint's target in its base units as a double.
     */
    private Setpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
      this.applier = applier;
      this.mode = mode;
      this.baseUnits = baseUnits;
    }

    /**
     * Creates a completely custom Setpoint with a given applier, mode, and base units.
     *
     * @param applier The function that applies the setpoint to a MotorIO.
     * @param mode The control mode of the setpoint.
     * @param baseUnits The Setpoint's target in its base units as a double.
     */
    public static Setpoint withCustomSetpoint(
        UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
      return new Setpoint(applier, mode, baseUnits);
    }

    /**
     * Creates a setpoint to idle.
     *
     * @return A new Setpoint.
     */
    public static Setpoint withNeutralSetpoint() {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setNeutralSetpoint();
            return io;
          };
      return new Setpoint(applier, Mode.IDLE, 0.0);
    }

    /**
     * Creates a setpoint to run at a voltage.
     *
     * @param voltage Voltage to run at.
     * @return A new Setpoint.
     */
    public static Setpoint withVoltageSetpoint(Voltage voltage) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setVoltageSetpoint(voltage);
            return io;
          };
      return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
    }

    /**
     * Creates a setpoint to go to a velocity.
     *
     * @param velocity Velocity to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withVelocitySetpoint(AngularVelocity velocity) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setVelocitySetpoint(velocity);
            return io;
          };
      return new Setpoint(applier, Mode.VELOCITY, velocity.baseUnitMagnitude());
    }

    /**
     * Creates a setpoint to use PID control to go to a position.
     *
     * @param position Position to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withPositionSetpoint(Angle position) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setPositionSetpoint(position);
            return io;
          };
      return new Setpoint(applier, Mode.POSITION, position.baseUnitMagnitude());
    }

    /**
     * Creates a setpoint to run at a percent of max voltage.
     *
     * @param percent Percent of max voltage to run at.
     * @return A new Setpoint.
     */
    public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setDutyCycleSetpoint(percent);
            return io;
          };
      return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
    }

    /**
     * Creates a setpoint to use MotionMagic or MAXMotion control to go to a velocity.
     *
     * @param velocity Velocity to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withSmartVelocitySetpoint(AngularVelocity velocity) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setSmartVelocitySetpoint(velocity);
            return io;
          };
      return new Setpoint(applier, Mode.SMART_VEL, velocity.baseUnitMagnitude());
    }

    /**
     * Creates a setpoint to use MotionMagic or MAXMotion control to go to a position.
     *
     * @param position Position to go to in mechanism units.
     * @return A new Setpoint.
     */
    public static Setpoint withSmartPositionSetpoint(Angle position) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setSmartPositionSetpoint(position);
            return io;
          };
      return new Setpoint(applier, Mode.SMART_POS, position.baseUnitMagnitude());
    }

    public void apply(MotorIO io) {
      applier.apply(io);
    }
  }
}
