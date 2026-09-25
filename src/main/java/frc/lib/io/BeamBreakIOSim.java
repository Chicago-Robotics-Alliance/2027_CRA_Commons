// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import edu.wpi.first.units.measure.Time;
import java.util.function.BooleanSupplier;

/**
 * Used for a simulated beam break sensor or other non-physical boolean input. For example, this
 * could be used to track when the current on a motor spikes above a certain threshold, indicating
 * that a game piece has been intaked.
 */
public class BeamBreakIOSim extends BeamBreakIO {
  private final BooleanSupplier button;

  public BeamBreakIOSim(BooleanSupplier buttonSupplier, Time debounce) {
    super(debounce);
    button = buttonSupplier;
  }

  @Override
  public void updateInputs() {
    inputs.state = button.getAsBoolean();
  }
}
