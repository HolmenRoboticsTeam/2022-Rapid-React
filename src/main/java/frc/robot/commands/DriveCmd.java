// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> getXAxis, getYAxis, getZRotation;

  public DriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> getYAxis, Supplier<Double> getXAxis, Supplier<Double> getZRotation) {
    this.driveSubsystem = driveSubsystem;
    this.getYAxis = getYAxis;
    this.getXAxis = getXAxis;
    this.getZRotation = getZRotation;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.driveSubsystem.arcadeDrive(-getXAxis.get(), getYAxis.get(), getZRotation.get());
  }

  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.arcadeDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
