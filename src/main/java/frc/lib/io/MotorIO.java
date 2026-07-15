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

/** Add your docs here. */
public abstract class MotorIO {
  public final AngleUnit unitType;
  public final TimeUnit time;
  protected final MotorIOInputs inputs;
  protected final MotorIOInputs[] followerInputs;
  private Setpoint setpoint = Setpoint.withNeutralSetpoint();
  private boolean enabled = true;

  public abstract void updateInputs();

  public abstract void setCurrentPosition(Angle position);

  public abstract void zeroSensors();

  public abstract void setNeutralBrake(boolean shouldBrake);

  public abstract void useSoftLimits(boolean enable);

  protected abstract void setNeutralSetpoint();

  protected abstract void setVoltageSetpoint(Voltage voltage);

  protected abstract void setVelocitySetpoint(AngularVelocity velocity);

  protected abstract void setPositionSetpoint(Angle position);

  protected abstract void setSmartVelocitySetpoint(AngularVelocity velocity);

  protected abstract void setSmartPositionSetpoint(Angle position);

  protected abstract void setDutyCycleSetpoint(Dimensionless percent);

  public final void applySetpoint(Setpoint desiredSetpoint) {
    setpoint = desiredSetpoint;
    if (enabled) {
      desiredSetpoint.apply(this);
    }
  }

  public final void enable() {
    enabled = true;
    setpoint.apply(this);
  }

  public final void disable() {
    enabled = false;
    Setpoint.withNeutralSetpoint().apply(this);
  }

  public boolean getEnabled() {
    return enabled;
  }

  protected MotorIO(AngleUnit unit, TimeUnit time) {
    this(unit, time, 0);
  }

  protected MotorIO(AngleUnit unit, TimeUnit time, int numFollowers) {
    this.unitType = unit;
    this.time = time;
    inputs = new MotorIOInputsAutoLogged();
    followerInputs = new MotorIOInputs[numFollowers];

    for (int i = 0; i < numFollowers; i++) {
      followerInputs[i] = new MotorIOInputsAutoLogged();
    }
  }

  public void processLogging(String name) {
    Logger.processInputs(name, (LoggableInputs) inputs);
    for (int i = 0; i < followerInputs.length; i++) {
      Logger.processInputs(name + "/Follower" + i, (LoggableInputs) followerInputs[i]);
    }
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public Angle getPosition() {
    return inputs.position;
  }

  public Current getStatorCurrent() {
    return inputs.statorCurrent;
  }

  public Current getSupplyCurrent() {
    return inputs.supplyCurrent;
  }

  public Voltage getMotorVoltage() {
    return inputs.motorVoltage;
  }

  public Setpoint getSetpoint() {
    return setpoint;
  }

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

  public enum Mode {
    IDLE,
    VOLTAGE,
    VELOCITY,
    POSITION,
    SMART_VEL,
    SMART_POS,
    DUTY_CYCLE;

    public boolean isPositionControl() {
      return switch (this) {
        case POSITION, SMART_POS -> true;
        default -> false;
      };
    }

    public boolean isVelocityControl() {
      return switch (this) {
        case VELOCITY, SMART_VEL -> true;
        default -> false;
      };
    }

    public boolean isNeutralControl() {
      return switch (this) {
        case IDLE -> true;
        default -> false;
      };
    }

    public boolean isVoltageControl() {
      return switch (this) {
        case VOLTAGE, DUTY_CYCLE -> true;
        default -> false;
      };
    }
  }

  public static class Setpoint {
    private final UnaryOperator<MotorIO> applier;
    public final Mode mode;
    public final double baseUnits;

    private Setpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
      this.applier = applier;
      this.mode = mode;
      this.baseUnits = baseUnits;
    }

    public static Setpoint withCustomSetpoint(
        UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
      return new Setpoint(applier, mode, baseUnits);
    }

    // withXSetpoint methods for each control mode
    public static Setpoint withNeutralSetpoint() {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setNeutralSetpoint();
            return io;
          };
      return new Setpoint(applier, Mode.IDLE, 0.0);
    }

    public static Setpoint withVoltageSetpoint(Voltage voltage) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setVoltageSetpoint(voltage);
            return io;
          };
      return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
    }

    public static Setpoint withVelocitySetpoint(AngularVelocity velocity) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setVelocitySetpoint(velocity);
            return io;
          };
      return new Setpoint(applier, Mode.VELOCITY, velocity.baseUnitMagnitude());
    }

    public static Setpoint withPositionSetpoint(Angle position) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setPositionSetpoint(position);
            return io;
          };
      return new Setpoint(applier, Mode.POSITION, position.baseUnitMagnitude());
    }

    public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setDutyCycleSetpoint(percent);
            return io;
          };
      return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
    }

    public static Setpoint withSmartVelocitySetpoint(AngularVelocity velocity) {
      UnaryOperator<MotorIO> applier =
          (io) -> {
            io.setSmartVelocitySetpoint(velocity);
            return io;
          };
      return new Setpoint(applier, Mode.SMART_VEL, velocity.baseUnitMagnitude());
    }

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
