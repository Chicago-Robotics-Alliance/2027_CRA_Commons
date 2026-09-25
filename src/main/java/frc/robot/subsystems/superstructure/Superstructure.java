// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem manages all other mechanism subsystems on the robot. The state machine handles
 * transitions between robot states and sets the setpoints of the individual subsystems. They should
 * not have their setpoints modified outside of the state machine logic. This subsystem should
 * expose commands for actions that involve the whole robot, i.e. "score game piece", which can be
 * bound to triggers.
 */
public class Superstructure extends SubsystemBase {
  public static final Superstructure mInstance = new Superstructure();

  private SuperstructureState state = SuperstructureState.IDLE;

  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state.toString());
  }

  // Add commands here.

  /**
   * Use this enum to represent the possible states of the robot's mechanisms. REBUILT example:
   * IDLE, INTAKING, SCORING, PASSING
   */
  public enum SuperstructureState {
    IDLE,
  }
}
