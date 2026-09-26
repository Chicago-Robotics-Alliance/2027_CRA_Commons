// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Setpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Base subsystem for any subsystem that uses motors. */
public class MotorSubsystem<IO extends MotorIO> extends SubsystemBase {
  protected final IO io;
  protected final String name;
  protected final double gearRatio;

  /**
   * Creates a MotorSubsystem with a MotorIO and name for telemetry.
   *
   * @param io MotorIO for the subsystem.
   * @param gearRatio Gear ratio for the subsystem.
   * @param name Name for telemetry.
   */
  public MotorSubsystem(IO io, String name, double gearRatio) {
    super(name);
    this.io = io;
    this.name = name;
    this.gearRatio = gearRatio;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    io.processLogging(name);
    Logger.recordOutput(name + "/Setpoint/Mode", io.getSetpoint().mode.toString());
    Logger.recordOutput(name + "/Setpoint/Value", io.getSetpoint().baseUnits);
  }

  /**
   * Gets the last read position of the subsystem's main motor.
   *
   * @return Position of the subsystem, in mechanism units (i.e. accounting for gear ratio).
   */
  public Angle getPosition() {
    return io.getPosition().div(gearRatio);
  }

  /**
   * Gets the last read velocity of the subsystem's main motor.
   *
   * @return Velocity of the subsystem, in mechanism units (i.e. accounting for gear ratio).
   */
  public AngularVelocity getVelocity() {
    return io.getVelocity().div(gearRatio);
  }

  /**
   * Gets the last read stator current of the subsystem's main motor.
   *
   * @return Stator current of the subsystem.
   */
  public Current getStatorCurrent() {
    return io.getStatorCurrent();
  }

  /**
   * Gets the last read supply current of the subsystem's main motor.
   *
   * @return Supply current of the subsystem.
   */
  public Current getSupplyCurrent() {
    return io.getSupplyCurrent();
  }

  /**
   * Gets the last read output voltage of the subsystem's main motor.
   *
   * @return Output voltage of the subsystem.
   */
  public Voltage getMotorVoltage() {
    return io.getMotorVoltage();
  }

  /**
   * Gets the last applied setpoint to the MotorIO.
   *
   * @return Last applied setpoint.
   */
  public Setpoint getSetpoint() {
    return io.getSetpoint();
  }

  /**
   * Applies a setpoint to the MotorIO.
   *
   * @param setpoint Setpoint to apply.
   */
  public void applySetpoint(Setpoint setpoint) {
    io.applySetpoint(setpoint);
  }

  /**
   * Creates a one-time command for the subsystem to go to a given Setpoint.
   *
   * @param setpoint Setpoint to go to.
   * @return One-time Command for the subsystem.
   */
  public Command setpointCommand(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint));
  }

  /**
   * Creates a continuous command for the subsystem to repeatedly go to a supplied Setpoint.
   *
   * @param supplier Supplier of Setpoint to go to.
   * @return Continuous Command for the subsystem.
   */
  public Command followSetpointCommand(Supplier<Setpoint> supplier) {
    return run(() -> applySetpoint(supplier.get()));
  }

  /**
   * Disables this Subsystem's MotorIO. Setpoints can still be set when disabled but will not be
   * applied until re-enabled.
   */
  public void disable() {
    io.disable();
  }

  /**
   * Enables this Subsystem's MotorIO. Immediately applies the last set setpoint including setpoints
   * set when disabled. MotorIO is enabled by default.
   */
  public void enable() {
    io.enable();
  }

  /**
   * Creates a one-time command for the subsystem to disable the MotorIO.
   *
   * @return A one-time Command to disable the MotorIO. Does not require the subsystem.
   */
  public Command disableCommand() {
    return runOnce(io::disable);
  }

  /**
   * Creates a one-time command for the subsystem to enable the MotorIO.
   *
   * @return A one-time Command to enable the MotorIO. Does not require the subsystem.
   */
  public Command enableCommand() {
    return runOnce(io::enable);
  }
}
