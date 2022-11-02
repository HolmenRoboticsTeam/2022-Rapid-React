// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimbersCommand extends CommandBase {
  // Subsystems
  private ClimberSubsystem climberSubsystem;

  /** Creates a new ExtendClimberCommand. */
  public ExtendClimbersCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

    this.addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.climberSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.climberSubsystem.setLeftClimber(ClimberConstants.kExtendSpeed);
    this.climberSubsystem.setRightClimber(ClimberConstants.kExtendSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
