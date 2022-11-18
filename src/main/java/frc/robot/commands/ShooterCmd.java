// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShooterCmd extends ParallelCommandGroup {   // button type parallels with the management system

  private ShooterFlywheelSubsystem shooterSubsystem;
  private LimeLightSubsystem limeLightSubsystem;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private ShooterFlywheelSubsystem shooterFlywheelSubsystem;
  private double speed;
  Joystick stick; // temporary

  public ShooterCmd(ShooterFlywheelSubsystem shooterSubsystem, LimeLightSubsystem limeLightSubsystem, ShooterAngleSubsystem shooterAngleSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, double speed, Joystick stick) {

    this.shooterSubsystem = shooterSubsystem;
    this.limeLightSubsystem = limeLightSubsystem;
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
    this.speed = speed;
    this.stick = stick;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double velocityNeeded = limeLightSubsystem.exitVelocityNeededInMeters();
    if (limeLightSubsystem.getDistanceToTargetMeters() < 3) {
      velocityNeeded -= 3;
    } else if (limeLightSubsystem.getDistanceToTargetMeters() >= 3.5) {
      velocityNeeded -= 4;
    } else {
      velocityNeeded -= 1;
    }
    shooterFlywheelSubsystem.setFrontMotorVelocity(velocityNeeded);
    shooterFlywheelSubsystem.setBackMotorVelocity(velocityNeeded);
    // System.out.println((this.stick.getRawAxis(3)));  // TODO: Remove
    // shooterSubsystem.setMotor(speed); // Replace with ShooterConstants.kSpeed
    //shooterSubsystem.setMotor(stick.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
