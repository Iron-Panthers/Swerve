// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

/**
 * A non-Subsystem class which represents the logic of an individual swerve
 * module (meaning there are 4 instances).
 * 
 * It contains all hardware (drive motor, turn motor, encoder), as well as PID
 * controllers for the motors.
 * 
 * State is read from getState, and updated through setDesiredState /
 * setNeutralState.
 */
public class SwerveDriveModule {
  // Hardware
  /**
   * TalonFX object for the motor which drives the wheel.
   */
  private final TalonFX driveMotor;
  /**
   * TalonSRX object for the motor which adjusts the wheel angle.
   */
  private final TalonSRX turnMotor;
  /**
   * CAN Mag Encoder object for the encoder which reads the current wheel angle.
   */
  private final CANCoder turnEncoder;

  /**
   * Profiled PID Controller for the wheel speed. This is used when updating the
   * desired velocity of the wheel.
   * 
   * "Profiled" means that the velocity and acceleration limits can be defined.
   */
  private final ProfiledPIDController drivePIDController = new ProfiledPIDController(ModuleConstants.kP_drive, 0, 0,
      new TrapezoidProfile.Constraints(ModuleConstants.kAngularVelocityLimit,
          ModuleConstants.kAngularAccelerationLimit));

  /**
   * PID Controller for the wheel angle. This is used when updating the desired
   * angle of the module.
   */
  private final PIDController turningPIDController = new PIDController(ModuleConstants.kP_turn, 0, 0);

  /**
   * A SwerveModuleState object for storing the most recent goal state.
   * 
   * This is used for Shuffleboard logging and to get the angle goal for
   * setNeutralState.
   */
  private SwerveModuleState goalState = new SwerveModuleState();

  public SwerveDriveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort, boolean driveEncoderReversed,
      boolean turningEncoderReversed, String name, final double encoderOffset) {
    // Contains the configurations to apply to all modules
    this.driveMotor = new TalonFX(driveMotorPort);
    driveMotor.configFactoryDefault();
    driveMotor.setSelectedSensorPosition(0);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setInverted(driveEncoderReversed);

    this.turnMotor = new TalonSRX(turnMotorPort);
    turnMotor.configFactoryDefault();
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.setInverted(turningEncoderReversed);

    this.turnEncoder = new CANCoder(turnEncoderPort);
    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    turnEncoder.configMagnetOffset(0 - encoderOffset, 10 /* ms */);

    this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // LOGGING
    Shuffleboard.getTab(name).addString("current_state_string", () -> this.getState().toString());
    Shuffleboard.getTab(name).addString("goal_state_string", () -> goalState.toString());
    Shuffleboard.getTab(name).addNumber("drive_position_native_units", () -> driveMotor.getSelectedSensorPosition());
    Shuffleboard.getTab(name).addNumber("drive_position_converted",
        () -> driveMotor.getSelectedSensorPosition() / ModuleConstants.kTicksToMeters);
    Shuffleboard.getTab(name).add("drive pid", drivePIDController);
    Shuffleboard.getTab(name).add("turn pid", turningPIDController);
  }

  /**
   * Get the driving velocity of the module in meters per second.
   * 
   * @return Drive motor integrated sensor velocity, converted to meters per
   *         second.
   */
  private double getWheelSpeed() {
    return driveMotor.getSelectedSensorVelocity() / ModuleConstants.kTicksToMeters;
  }

  /**
   * Get the absolute position of the wheel in degrees.
   * 
   * @return Turn encoder reading converted into a Rotation2d object.
   */
  private Rotation2d getWheelAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
  }

  /**
   * Return the current state of the module (velocity, angle). Uses
   * getWheelSpeed() and getWheelAngle().
   * 
   * @return SwerveModuleState object representing the module state.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getWheelSpeed(), getWheelAngle());
  }

  /**
   * Update the desired swerve module state.
   * 
   * The module may approach the specified state or its "inverse" equivalent
   * state. See SwerveModuleState.optimize() for more details.
   * 
   * @param desired The reference for the swerve module to move towards.
   */
  public void setDesiredState(SwerveModuleState desired) {
    /* Use optimize method to choose ideal module state */
    // In certain cases, it may be advantageous to approach an angle rotated by 180
    // degrees and flip the wheel speed.
    goalState = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition()));

    /* Calculate controller outputs */
    // Do comparisons between current velocity/angle and the new goal state
    // velocity/angle
    final double driveOutput = drivePIDController.calculate(getWheelSpeed(), goalState.speedMetersPerSecond);
    final var turningOutput = turningPIDController.calculate(getWheelAngle().getRadians(),
        goalState.angle.getRadians());

    // Write controller outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    turnMotor.set(ControlMode.PercentOutput, turningOutput);
  }

  /**
   * Stop the wheel while maintaining PID control of the wheel angle.
   */
  public void setNeutralState() {
    /* Calculate controller outputs */
    // Do comparison between current velocity and zero velocity (stopped)
    final double driveOutput = drivePIDController.calculate(getWheelSpeed(), 0);
    // Do comparison between current angle and cached goal state angle
    final var turningOutput = turningPIDController.calculate(getWheelAngle().getRadians(),
        goalState.angle.getRadians());

    // Write controller outputs
    driveMotor.set(ControlMode.PercentOutput, driveOutput);
    turnMotor.set(ControlMode.PercentOutput, turningOutput);
  }
}
