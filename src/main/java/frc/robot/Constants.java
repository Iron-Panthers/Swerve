// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/** Class to contain all constants */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kRearRightDriveMotorPort = 12;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearLeftTurningMotorPort = 4;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int kFrontLeftTurningEncoderPort = 12;
    public static final int kFrontRightTurningEncoderPort = 9;
    public static final int kRearLeftTurningEncoderPort = 10;
    public static final int kRearRightTurningEncoderPort = 11;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveReversed = false;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kRearLeftDriveReversed = false;
    public static final boolean kRearRightDriveReversed = true;

    private static final double kDistanceBetweenWheels = 2 * 0.1016;
    public static final double kTrackWidth = kDistanceBetweenWheels;
    public static final double kWheelBase = kDistanceBetweenWheels;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 0.5; // METERS PER SECOND (CONFIRMED)
  }

  public static final class ModuleConstants {
    public static final double kAngularVelocityLimit = 2 * Math.PI; // rad/s
    public static final double kAngularAccelerationLimit = 2 * Math.PI; // rad/s2

    public static final double kTicksToMeters = 26666.666;

    public static final double kTurningEncoderDistancePerPulse = 0;
    public static final double kDriveEncoderDistancePerPulse = 0;

    // NOT tuned
    public static final double kP_turn = 0.5;
    public static final double kP_drive = 1;

    // angles at effective module 0 for absolute encoders
    // Since magnetic encoders read some arbitrary value at "true zero"
    // fl, fr, bl, br
    public static final double kModuleOffsets[] = { 315.18, 265.3, 63.4, 320.4 };
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
