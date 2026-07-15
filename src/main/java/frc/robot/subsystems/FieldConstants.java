// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
  private static Pose2d makeTarget(double x, double y) {
    return new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
  }

  public static final double FIELD_LENGTH = 16.5405;
  public static final double FIELD_WIDTH = 8.0695;

  // Boundaries for zones of the field, used to determine which target to aim at
  public static final double BLUE_ALLIANCE_LINE_X = 4.4;
  public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH - 4.4;
  public static final double CENTER_LINE_Y = FIELD_WIDTH / 2;

  // Targets for the shooter to aim at
  public static final Pose2d BLUE_HUB = makeTarget(4.625, CENTER_LINE_Y);
  public static final Pose2d RED_HUB = makeTarget(FIELD_LENGTH - 4.625, CENTER_LINE_Y);
  // public static final Pose2d BLUE_SIM_START = makeTarget(3.57, CENTER_LINE_Y);
  // public static final Pose2d RED_SIM_START = makeTarget(FIELD_LENGTH-3.57, CENTER_LINE_Y);
  public static final Pose2d BLUE_PASS_OUTPOST = makeTarget(0, 1);
  public static final Pose2d BLUE_PASS_DEPOT = makeTarget(0, FIELD_WIDTH - 1);
  public static final Pose2d RED_PASS_OUTPOST = makeTarget(FIELD_LENGTH - 0, FIELD_WIDTH - 1);
  public static final Pose2d RED_PASS_DEPOT = makeTarget(FIELD_LENGTH - 0, 1);

  // Auto starting positions, used to reset odometry at the beginning of auto
  public static final Pose2d BLUE_START_DEPOT = new Pose2d(4, 7.25, Rotation2d.fromDegrees(-79));
  public static final Pose2d BLUE_START_OUTPOST =
      new Pose2d(4, FIELD_WIDTH - 7.25, Rotation2d.fromDegrees(79));
  public static final Pose2d RED_START_OUTPOST =
      new Pose2d(FIELD_LENGTH - 4, 7.25, Rotation2d.fromDegrees(-101));
  public static final Pose2d RED_START_DEPOT =
      new Pose2d(FIELD_LENGTH - 4, FIELD_WIDTH - 7.25, Rotation2d.fromDegrees(101));
}
