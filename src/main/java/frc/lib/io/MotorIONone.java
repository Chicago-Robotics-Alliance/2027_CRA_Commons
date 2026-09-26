// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.io;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

/** No-op MotorIO for replay mode, where no real hardware should be touched. */
public class MotorIONone extends MotorIO {
  public MotorIONone() {
    super(Units.Rotations, Units.Seconds);
  }

  @Override
  public void updateInputs() {}

  @Override
  public void setCurrentPosition(Angle position) {}

  @Override
  public void zeroSensors() {}

  @Override
  public void setNeutralBrake(boolean shouldBrake) {}

  @Override
  public void useSoftLimits(boolean enable) {}

  @Override
  protected void setNeutralSetpoint() {}

  @Override
  protected void setVoltageSetpoint(Voltage voltage) {}

  @Override
  protected void setVelocitySetpoint(AngularVelocity velocity) {}

  @Override
  protected void setPositionSetpoint(Angle position) {}

  @Override
  protected void setSmartVelocitySetpoint(AngularVelocity velocity) {}

  @Override
  protected void setSmartPositionSetpoint(Angle position) {}

  @Override
  protected void setDutyCycleSetpoint(Dimensionless percent) {}
}
