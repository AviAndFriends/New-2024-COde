package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kDrivingMotorGearRatio = 1 / 5.5;
    public static final double kTurningMotorGearRatio = 1 / 5.5;
    public static final double kDriveEncoderRot2Meter =
        kDrivingMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  /*change the units to fit our robot */
  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(21);
    /* distance between right and left wheels */
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    /*Distance between front and back wheels */
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), //
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 7;
    public static final int kBackRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 9;
    public static final int kFrontRightTurningMotorPort = 8;
    public static final int kBackRightTurningMotorPort = 8;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 0;
    public static final TrapezoidProfile.Constraints kThetaControllerConstants = //
        new TrapezoidProfile.Constraints( //
            kTeleDriveMaxAngularSpeedRadiansPerSecond, //
            kMaxAngularAccelerationRadiansPerSecondSquared //
            );
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}