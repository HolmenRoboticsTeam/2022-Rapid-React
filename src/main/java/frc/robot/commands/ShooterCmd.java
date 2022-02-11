// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends ParallelCommandGroup {
  // button type parallels with the management system
  private final ShooterSubsystem ShooterSubsystem;
  private final boolean on;
  /** Creates a new ShooterCmd. */
  public ShooterCmd(ShooterSubsystem shooterSubsystem, boolean on) {

    this.ShooterSubsystem = shooterSubsystem;
    this.on = on;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.ShooterOn(on);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
