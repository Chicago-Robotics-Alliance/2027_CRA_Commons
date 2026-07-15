// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

/** Add your docs here. */
public class Util {
  public static final double kEpsilon = 1E-12;

  private Util() {}

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static int limit(int v, int min, int max) {
    return Math.min(max, Math.max(min, v));
  }

  public static boolean inRange(double v, double maxMagnitude) {
    return inRange(v, -maxMagnitude, maxMagnitude);
  }

  /** Checks if the given input is within the range (min, max), both exclusive. */
  public static boolean inRange(double v, double min, double max) {
    return v > min && v < max;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static String joinStrings(final String delim, final List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(Translation2d a, Translation2d b) {
    return epsilonEquals(a.getX(), b.getX()) || epsilonEquals(a.getY(), b.getY());
  }

  public static boolean epsilonEquals(Translation2d a, Translation2d b, double epsilon) {
    return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
  }

  public static boolean epsilonEquals(Translation2d a, Translation2d b, Distance epsilon) {
    return epsilonEquals(a.getX(), b.getX(), epsilon.in(Units.Meters))
        && epsilonEquals(a.getY(), b.getY(), epsilon.in(Units.Meters));
  }

  public static boolean epsilonEquals(ChassisSpeeds a, ChassisSpeeds b) {
    return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond)
        && epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond)
        && epsilonEquals(a.omegaRadiansPerSecond, b.omegaRadiansPerSecond);
  }

  public static boolean epsilonEquals(
      ChassisSpeeds a, ChassisSpeeds b, double linearVelocityEpsilon) {
    return epsilonEquals(a.vxMetersPerSecond, b.vxMetersPerSecond, linearVelocityEpsilon)
        && epsilonEquals(a.vyMetersPerSecond, b.vyMetersPerSecond, linearVelocityEpsilon);
  }

  /**
   * Class used store translate distances in the form of angles. Used for elevators to interface
   * with the IO layer which only supports angles.
   */
  public static class DistanceAngleConverter {
    private final Distance radius;

    public DistanceAngleConverter(Distance radius) {
      this.radius = radius;
    }

    /**
     * Converts a distance measurement to an equal angle measurement based on radius initialized
     * with.
     *
     * @param distance Distance to convert to angle.
     * @return Angle distance is equivalent to.
     */
    public Angle toAngle(Distance distance) {
      return Units.Radians.of(distance.in(BaseUnits.DistanceUnit) / radius.baseUnitMagnitude());
    }

    /**
     * Converts an angle measurement to an equal distance measurement based on radius initialized
     * with.
     *
     * @param distance angle to convert to distance.
     * @return Distance agle is equivalent to.
     */
    public Distance toDistance(Angle angle) {
      return BaseUnits.DistanceUnit.of(angle.in(Units.Radians) * radius.baseUnitMagnitude());
    }

    /**
     * Gets an angle unit equivalent to a distance unit with the conversion of the radius
     * initialized with.
     *
     * @param unit The distance unit to convert.
     * @return The distance represented as an AngleUnit
     */
    public AngleUnit getDistanceUnitAsAngleUnit(DistanceUnit unit) {
      return Units.derive(BaseUnits.AngleUnit)
          .aggregate(toAngle(unit.one()).baseUnitMagnitude())
          .named(unit.name())
          .symbol(unit.symbol())
          .make();
    }

    /**
     * Gets a distance unit equivalent to a angle unit with the conversion of the radius initialized
     * with.
     *
     * @param unit The angle unit to convert.
     * @return The distance represented as a DistanceUnit
     */
    public DistanceUnit getAngleUnitAsDistanceUnit(AngleUnit unit) {
      return Units.derive(BaseUnits.DistanceUnit)
          .splitInto(toDistance(unit.one()).baseUnitMagnitude())
          .named(unit.name())
          .symbol(unit.symbol())
          .make();
    }

    public Distance getDrumRadius() {
      return radius;
    }
  }
}
