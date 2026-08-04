// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure extends SubsystemBase {
  public static final Superstructure mInstance = new Superstructure();
  
  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Returns a command that spins up the shooter to the close setpoint, then enables the indexer/intake when the shooter is spun up
  public Command closeShot() {
    return Shooter.mInstance.followSetpointCommand(() -> Shooter.CLOSE_SHOT)
    .until(Shooter.mInstance::spunUp)
    .andThen(Commands.parallel(
      Shooter.mInstance.followSetpointCommand(() -> Shooter.CLOSE_SHOT)
      // Indexer.mInstance.followSetpointCommand(() -> Indexer.FEEDING),
      // Intake.mInstance.followSetpointCommand(() -> Intake.INTAKING)
    ));
  }

  // Returns a command that spins up the shooter to the medium setpoint, then enables the indexer/intake when the shooter is spun up
  public Command mediumShot() {
    return Shooter.mInstance.followSetpointCommand(() -> Shooter.MEDIUM_SHOT)
    .until(Shooter.mInstance::spunUp)
    .andThen(Commands.parallel(
      Shooter.mInstance.followSetpointCommand(() -> Shooter.MEDIUM_SHOT)
      // Indexer.mInstance.followSetpointCommand(() -> Indexer.FEEDING),
      // Intake.mInstance.followSetpointCommand(() -> Intake.INTAKING)
    ));
  }

  // Returns a command that spins up the shooter to the far setpoint, then enables the indexer/intake when the shooter is spun up
  public Command farShot() {
    return Shooter.mInstance.followSetpointCommand(() -> Shooter.FAR_SHOT)
    .until(Shooter.mInstance::spunUp)
    .andThen(Commands.parallel(
      Shooter.mInstance.followSetpointCommand(() -> Shooter.FAR_SHOT)
      // Indexer.mInstance.followSetpointCommand(() -> Indexer.FEEDING),
      // Intake.mInstance.followSetpointCommand(() -> Intake.INTAKING)
    ));
  }

  // Returns a command that sets all subsystems to idle
  public Command idle() {
    return Commands.sequence(
      // Indexer.mInstance.setpointCommand(Indexer.IDLE),
      // Intake.mInstance.setpointCommand(Intake.IDLE),
      Shooter.mInstance.setpointCommand(Shooter.IDLE)
    );
  }

}
