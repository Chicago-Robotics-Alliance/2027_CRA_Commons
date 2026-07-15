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

public class MotorSubsystem<IO extends MotorIO> extends SubsystemBase {
  protected final IO io;
  protected final String name;

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem(IO io, String name) {
    super(name);
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    io.processLogging(name);
  }

  public Angle getPosition() {
    return io.getPosition();
  }

  public AngularVelocity getVelocity() {
    return io.getVelocity();
  }

  public Current getStatorCurrent() {
    return io.getStatorCurrent();
  }

  public Current getSupplyCurrent() {
    return io.getSupplyCurrent();
  }

  public Voltage getMotorVoltage() {
    return io.getMotorVoltage();
  }

  public Setpoint getSetpoint() {
    return io.getSetpoint();
  }

  public void applySetpoint(Setpoint setpoint) {
    io.applySetpoint(setpoint);
  }

  public Command setpointCommand(Setpoint setpoint) {
    return runOnce(() -> applySetpoint(setpoint));
  }

  public Command followSetpointCommand(Supplier<Setpoint> supplier) {
    return run(() -> applySetpoint(supplier.get()));
  }

  public void disable() {
    io.disable();
  }

  public void enable() {
    io.enable();
  }

  public Command disableCommand() {
    return runOnce(io::disable);
  }

  public Command enableCommand() {
    return runOnce(io::enable);
  }
}
