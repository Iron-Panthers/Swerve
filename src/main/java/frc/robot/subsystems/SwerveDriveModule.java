// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

/**
 * Importantly, not a subsystem, this class represents the logic of an
 * individual swerve module (meaning there are 4 instances).
 * 
 * It contains all hardware (drive motor, turn motor, encoder), as well as PID
 * controllers for the motors. It also takes in the absolute encoder's angle at
 * 0 reading (the offset value stored in Constants).
 * 
 * 
 */
public class SwerveDriveModule {
  private final TalonFX driveMotor;
  private final TalonSRX turnMotor;
  private final CANCoder turnEncoder;

  private final ProfiledPIDController drivePIDController = new ProfiledPIDController(ModuleConstants.kP_drive, 0, 0,
      new TrapezoidProfile.Constraints(ModuleConstants.kAngularVelocityLimit,
          ModuleConstants.kAngularAccelerationLimit));

  private final PIDController turningPIDController = new PIDController(ModuleConstants.kP_turn, 0, 0);

  private final String name;
  private final double encoderOffsetDegrees;

  public SwerveDriveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, boolean driveEncoderReversed,
      boolean turningEncoderReversed, String name, final double encoderOffset) {
    this.driveMotor = new TalonFX(driveMotorPort);
    driveMotor.setSelectedSensorPosition(0);
    driveMotor.configFactoryDefault();
    driveMotor.setInverted(driveEncoderReversed);

    this.turnMotor = new TalonSRX(turnMotorPort);
    turnMotor.configFactoryDefault();
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.setInverted(turningEncoderReversed);

    this.turnEncoder = new CANCoder(turnEncoderPort);
    turnEncoder.setPosition(0, 10);

    // NOTE: did config through tuner app instead but these should be the equivalent
    // lines
    // turnEncoder.configFactoryDefault();
    // turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,
    // 10);

    this.name = name;

    encoderOffsetDegrees = encoderOffset;
  }

  // m/s
  // Returns the true velocity in m/s
  private double getRealDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() / ModuleConstants.kTicksToMeters;
  }

  // Returns the "true" angle of the module (todo: consider writing offset into encoder?)
  private Rotation2d getRealTurnPos() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition() - encoderOffsetDegrees);
  }

  /**
   * @return SwerveModuleState representing the module's current velocity and
   *         wheel angle.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getRealDriveVelocity(), getRealTurnPos());
  }

  /**
   * Returns a target swerve module state that is either exactly equal to the
   * parameter or flipped in direction and magnitude, depending on which is easier
   * to reach from the current module angle.
   */
  private SwerveModuleState optimize(SwerveModuleState state, Rotation2d angle) {
    var da = state.angle.minus(angle);
    if (Math.abs(da.getDegrees()) > 90) { // flip
      return new SwerveModuleState(-state.speedMetersPerSecond, state.angle.rotateBy(Rotation2d.fromDegrees(180)));
    }
    return state;
  }

  // Method which is called to update the swerve module state
  public void setDesiredState(SwerveModuleState desired) {
    SwerveModuleState st = optimize(desired, new Rotation2d(turnEncoder.getAbsolutePosition()));

    final double driveOutput = drivePIDController.calculate(getRealDriveVelocity(), st.speedMetersPerSecond);
    final var turningOutput = turningPIDController.calculate(getRealTurnPos().getRadians(), st.angle.getRadians());

    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    turnMotor.set(ControlMode.PercentOutput, turningOutput);
  }

  // For logging/debugging purposes. Called in DriveSubsystem, can be commented
  // out if it causes issues
  public void printInfo() {
    SmartDashboard.putString(name + " state", getState().toString());
    SmartDashboard.putNumber(name + "drive motor ticks", driveMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(name + " drive motor pos",
        driveMotor.getSelectedSensorPosition() / ModuleConstants.kTicksToMeters);
  }
}
