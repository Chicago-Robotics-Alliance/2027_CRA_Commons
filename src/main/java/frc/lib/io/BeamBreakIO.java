// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

/**
 * Represents a beam break sensor, limit switch, or other boolean input with debouncing
 * capabilities.
 */
public abstract class BeamBreakIO {
  private final Debouncer debouncer;
  protected final BeamBreakIOInputs inputs = new BeamBreakIOInputs();

  public BeamBreakIO(Time debounce) {
    debouncer = new Debouncer(debounce.in(Units.Seconds), DebounceType.kBoth);
  }

  public abstract void updateInputs();

  public boolean get() {
    return inputs.state;
  }
  ;

  public boolean getInverted() {
    return !get();
  }

  public boolean getDebounced() {
    return debouncer.calculate(get());
  }

  public boolean getDebouncedIfReal() {
    return Robot.isReal() && getDebounced();
  }

  public Command stateWait(boolean state) {
    return Commands.waitUntil(() -> get() == state);
  }

  public Command stateWaitWithDebounce(boolean state) {
    return Commands.waitUntil(() -> getDebounced() == state);
  }

  public Command stateWaitIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWait(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }

  public Command stateWaitWithDebounceIfReal(boolean state, double waitSecondsSim) {
    return Commands.either(
        stateWaitWithDebounce(state), Commands.waitSeconds(waitSecondsSim), () -> Robot.isReal());
  }

  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean state = false;
  }
}
