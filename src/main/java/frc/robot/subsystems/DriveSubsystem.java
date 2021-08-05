package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * Contains the subsystem logic for all swerve drive modules, as well as the
 * robot AHRS.
 * 
 * Responsible for regulating odometry, as well as updating any setpoints for
 * all of the modules.
 * 
 * In teleoperated logic, use {@link #drive(double, double, double, boolean)}
 * with the respective axis/button values as parameters. In autonomous logic,
 * you can supply swerve module states directly with
 * {@link #setModuleStates(SwerveModuleState[])}.
 */
public class DriveSubsystem extends SubsystemBase {
  /**
   * Front left swerve module
   */
  private final SwerveDriveModule frontLeft = new SwerveDriveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveReversed, DriveConstants.kFrontLeftTurningEncoderReversed, "FL",
      ModuleConstants.kModuleOffsets[0]);

  /**
   * Front right swerve module
   */
  private final SwerveDriveModule frontRight = new SwerveDriveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveReversed, DriveConstants.kFrontRightTurningEncoderReversed, "FR",
      ModuleConstants.kModuleOffsets[1]);

  /**
   * Rear left swerve module
   */
  private final SwerveDriveModule rearLeft = new SwerveDriveModule(DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort, DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveReversed, DriveConstants.kRearLeftTurningEncoderReversed, "BL",
      ModuleConstants.kModuleOffsets[2]);

  /**
   * Rear right swerve module
   */
  private final SwerveDriveModule rearRight = new SwerveDriveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort, DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveReversed, DriveConstants.kRearRightTurningEncoderReversed, "BR",
      ModuleConstants.kModuleOffsets[3]);

  /**
   * Robot gyro
   */
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  /**
   * SwerveDriveOdometry object which is used to track the robot's position on the
   * field over the course of the match.
   * 
   * Must be initialized with correct kinematics object and accurate initial gyro
   * angle to function.
   */
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d());

  public DriveSubsystem() {
    // LOGGING
    Shuffleboard.getTab("DriveSubsystem").addBoolean("initialized", () -> true);
    Shuffleboard.getTab("DriveSubsystem").addNumber("gyro_angle", gyro::getAngle);
    Shuffleboard.getTab("DriveSubsystem").addString("odometry_string", () -> odometry.getPoseMeters().toString());
  }

  @Override
  public void periodic() {
    // Update position estimate using current robot angle and module states
    // Module states MUST match with the order in kDriveKinematics in Constants.java
    odometry.update(new Rotation2d(getHeading()), // robot angle
        frontLeft.getState(), // corresponds to kDriveKinematics[0]
        rearLeft.getState(), // [1]
        frontRight.getState(), // [2]
        rearRight.getState() // [3]
    );
  }

  /**
   * Returns the current estimated position of the robot.
   * 
   * @return Pose2d of the current estimated position.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Reset the position estimate of the robot to a specified pose.
   * 
   * @param pose The reference Pose2d object to reset the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  // telop drive control
  /**
   * Method for teleoperated driver control of the swerve drivebase.
   * 
   * @param xSpeed        X speed of the robot. Should be an axis value, in -1 to
   *                      +1
   * @param ySpeed        Y speed of the robot. Should be an axis value, in -1 to
   *                      +1
   * @param rot           Rotation speed of the robot. Should be an axis value, in
   *                      -1 to +1
   * @param fieldRelative Defines if the robot should be controlled in reference
   *                      frame relative to the field or its center. TODO detailed
   *                      explanation?
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Writes desired states to each of the swerve modules.
   * 
   * @param desiredStates Array of SwerveModuleState (essentially a "data class"
   *                      of velocity and module angle)
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Writes neutral states to each of the swerve modules.
   * 
   * This holds the angle of each indiviudal module but sets the desired speed of the wheel to 0.
   */
  public void setNeutral() {
    frontLeft.setNeutralState();
    frontRight.setNeutralState();
    rearLeft.setNeutralState();
    rearRight.setNeutralState();
  }

  /**
   * Reset the yaw of the gyro.
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Get the angle of the robot.
   * 
   * @return Heading of the robot in degrees [-180 .. 180]
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Get the angular rate (degrees/s) of the robot.
   * 
   * @return Turn rate of the robot degrees per second
   */
  public double getTurnRate() {
    // TODO: I actually don't remember which way the turn rate is supposed to be
    // positive
    // this code is kind of old and this might also possibly be wrong/outdated?
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
