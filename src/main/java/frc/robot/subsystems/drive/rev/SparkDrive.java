package frc.robot.subsystems.drive.rev;

import static frc.robot.subsystems.drive.rev.DriveConstants.*;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;

public class SparkDrive extends DriveBase {

  public SparkDrive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
  }

  @Override
  protected void startOdometryThread() {
    SparkOdometryThread.getInstance().start();
  }

  @Override
  protected RobotConfig getPathPlannerConfig() {
    return ppConfig;
  }

  @Override
  public Translation2d[] getModuleTranslations() {
    return moduleTranslations;
  }

  @Override
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  @Override
  public double getDriveBaseRadiusMeters() {
    return driveBaseRadius;
  }
}
