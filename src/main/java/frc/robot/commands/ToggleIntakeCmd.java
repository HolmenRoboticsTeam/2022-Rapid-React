// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCmd extends CommandBase {

  private final IntakeSubsystem toggleIntakeSubsystem;
  private boolean open;

  public ToggleIntakeCmd(IntakeSubsystem toggleIntakeSubsystem) {

    this.open = false;
    this.toggleIntakeSubsystem = toggleIntakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(toggleIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.open = !this.open;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggleIntakeSubsystem.setPosition(open);
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
