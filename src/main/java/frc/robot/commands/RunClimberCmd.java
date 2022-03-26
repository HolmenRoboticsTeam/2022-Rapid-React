// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimberCmd extends CommandBase {
  private final ClimberSubsystem climberSubsystem;
  private final double speed;
  private Joystick stick;

  /** Creates a new ClimberCmd. */
  public RunClimberCmd(ClimberSubsystem climberSubsystem, double speed, Joystick stick) {
    this.climberSubsystem = climberSubsystem;
    this.speed = speed;
    this.stick = stick;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setMotor(stick.getRawAxis(3));
    // climberSubsystem.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
