// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;

public class LimeLightCmd extends CommandBase {
  private final LimeLightSubsystem limeLightSubsystem;

  public LimeLightCmd(LimeLightSubsystem limeLightSubsystem) {
    this.limeLightSubsystem = limeLightSubsystem;

    addRequirements(limeLightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {} //

  @Override
  public void execute() {

  } //


  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
