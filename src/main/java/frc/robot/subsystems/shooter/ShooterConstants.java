// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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


    public static final AngularVelocity kEpsilonThreshold = Units.RPM.of(10); // 10 RPM acceptable error

    public static SparkBaseConfig getSparkConfig() {
        SparkBaseConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kCoast);

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
