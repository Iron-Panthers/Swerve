// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ManualSwerveCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final BooleanSupplier fieldCentricSupplier;

  /**
   * Creates a new ManualSwerveCommand.
   * 
   * ManualSwerveCommand is backed by DriveSubsystem's drive(double, double,
   * double, boolean) method for teleoperated control.
   */
  public ManualSwerveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier, BooleanSupplier fieldCentricSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // deadzones
    // TODO: these are kind of weird
    double x = xSupplier.getAsDouble();
    if (Math.abs(x) < 0.2)
      x = 0;
    double y = ySupplier.getAsDouble();
    if (Math.abs(y) < 0.2)
      y = 0;
    double rot = rotSupplier.getAsDouble();
    if (Math.abs(rot) < 0.2)
      rot = 0;

    if (Math.abs(x) < 0.2 && Math.abs(y) < 0.2 && Math.abs(rot) < 0.2) {
      drive.setNeutral();
      return;
    }

    boolean fieldCentric = fieldCentricSupplier.getAsBoolean();
    drive.drive(x, y, rot, fieldCentric);
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
