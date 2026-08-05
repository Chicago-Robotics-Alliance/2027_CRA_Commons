// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.io.MotorIOSpark;
import frc.lib.io.MotorIOSpark.MotorIOSparkConfig;

/** Add your docs here. */
public class ShooterConstants {
    public static final int kRightMotorID = 6;
    public static final int kLeftMotorID = 3;

    // Example RPM setpoints
    public static final AngularVelocity kCloseShotRPM = Units.RPM.of(3000);
    public static final AngularVelocity kMediumShotRPM = Units.RPM.of(4000);
    public static final AngularVelocity kFarShotRPM = Units.RPM.of(5000);

    // PID constants (example)
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.2; // Static feedforward
    public static final double kV = 0.5; // Velocity feedforward

    public static final AngularVelocity kEpsilonThreshold = Units.RPM.of(10); // 10 RPM acceptable error

    public static SparkBaseConfig getSparkConfig() {
        SparkBaseConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kCoast);

        // Configure PID and feedforward for slot 2 (velocity control)
        config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot2);
        config.closedLoop.feedForward.kV(kV, ClosedLoopSlot.kSlot2);
        config.closedLoop.p(kP, ClosedLoopSlot.kSlot2);
        config.closedLoop.i(kI, ClosedLoopSlot.kSlot2);
        config.closedLoop.d(kD, ClosedLoopSlot.kSlot2);

        return config;
    }

    public static MotorIOSparkConfig getIOConfig() {
        MotorIOSparkConfig config = new MotorIOSparkConfig();

        config.mainConfig = getSparkConfig();
        config.mainID = kRightMotorID;

        config.followerConfig = getSparkConfig();
        config.followerIDs = new int[] {kLeftMotorID};
        config.followerOpposeMain = new boolean[] {true};

        config.unit = Units.Rotations;
        config.time = Units.Minute;

        return config;
    } 

    public static MotorIOSpark getMotorIO() {
        return new MotorIOSpark(getIOConfig());
    }
}
