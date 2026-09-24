// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOSpark;
import frc.lib.subsystems.FlywheelMotorSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FlywheelMotorSubsystem<MotorIOSpark> {
  public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
  public static final Setpoint CLOSE_SHOT =
      Setpoint.withVelocitySetpoint(ShooterConstants.kCloseShotRPM);
  public static final Setpoint MEDIUM_SHOT =
      Setpoint.withVelocitySetpoint(ShooterConstants.kMediumShotRPM);
  public static final Setpoint FAR_SHOT =
      Setpoint.withVelocitySetpoint(ShooterConstants.kFarShotRPM);

  public static final Shooter mInstance = new Shooter();

  /** Creates a new Shooter. */
  public Shooter() {
    super(ShooterConstants.getMotorIO(), "Shooter", ShooterConstants.kEpsilonThreshold);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.processLogging(name);

    Logger.recordOutput("Shooter/Setpoint/Mode", io.getSetpoint().mode.toString());
    Logger.recordOutput("Shooter/Setpoint/BaseUnits", io.getSetpoint().baseUnits);
  }
}
