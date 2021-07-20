package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SwerveDriveModule frontLeft = new SwerveDriveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveReversed, DriveConstants.kFrontLeftTurningEncoderReversed, "FL",
      ModuleConstants.kModuleOffsets[0]);

  private final SwerveDriveModule frontRight = new SwerveDriveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveReversed, DriveConstants.kFrontRightTurningEncoderReversed, "FR",
      ModuleConstants.kModuleOffsets[1]);

  private final SwerveDriveModule rearLeft = new SwerveDriveModule(DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort, DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveReversed, DriveConstants.kRearLeftTurningEncoderReversed, "BL",
      ModuleConstants.kModuleOffsets[2]);

  private final SwerveDriveModule rearRight = new SwerveDriveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort, DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveReversed, DriveConstants.kRearRightTurningEncoderReversed, "BR",
      ModuleConstants.kModuleOffsets[3]);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d());

  public DriveSubsystem() {
    SmartDashboard.putBoolean("DriveSubsystem#inititalized", true);
  }

  @Override
  public void periodic() {
    odometry.update(new Rotation2d(getHeading()), frontLeft.getState(), rearLeft.getState(), frontRight.getState(),
        rearRight.getState());

    SmartDashboard.putNumber("robotgyro", gyro.getAngle());

    frontLeft.printInfo();
    frontRight.printInfo();
    rearLeft.printInfo();
    rearRight.printInfo();
  }

  /**
   * Returns the current estimated position of the robot.
   * 
   * @return Pose2d of the current estimated position.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  // telop drive control
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Writes desired states to each of the swerve modules.
   * 
   * @param swerveModuleStates Array of SwerveModuleState (essentially a "data
   *                           class" of velocity and module angle)
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kSpeedLimit);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * @return Heading of the robot in degrees [-180 .. 180]
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * @return Turn rate of the robot degrees per second
   */
  public double getTurnRate() {
    // TODO: I actually don't remember which way the turn rate is supposed to be
    // positive
    // this code is kind of old and this might also possibly be wrong/outdated?
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
