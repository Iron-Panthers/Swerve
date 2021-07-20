// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveSideways extends CommandBase {
  private final DriveSubsystem drive;
  private final DoubleSupplier axisSpeedSupplier;

  /** Creates a new SideDriveways. */
  public DriveSideways(DriveSubsystem drive, DoubleSupplier axisSpeedSupplier) {
    this.drive = drive;
    this.axisSpeedSupplier = axisSpeedSupplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = axisSpeedSupplier.getAsDouble();
    drive.drive(speed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
