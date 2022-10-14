// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class AngleShooterCmd extends CommandBase {
  /** Creates a new AngleShooterCmd. */
  private final ShooterAngleSubsystem shooterAngleSubsystem;
  private PIDController pid = new PIDController(0.03, 0, 0.0005);
  private Supplier<Double> targetAngleSupplier;

  Joystick stick;  // temporary

  public AngleShooterCmd(ShooterAngleSubsystem shooterVerticalSubsystem, Supplier<Double> targetAngleSupplier, Joystick stick) {
    this.shooterAngleSubsystem = shooterVerticalSubsystem;
    this.targetAngleSupplier = targetAngleSupplier;
    this.stick = stick;

    this.pid.disableContinuousInput();

    addRequirements(shooterVerticalSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pid.reset();
    SmartDashboard.putNumber("Shooter Angle Minimum Speed", ShooterConstants.kMinimumRotationSpeed);
    SmartDashboard.putNumber("Shooter Angle Maximum Speed", ShooterConstants.kMaximumRotationSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("Angle", shooterAngleSubsystem.anglePerRotation());
    System.out.print("ANGLE: ");
    System.out.println(shooterAngleSubsystem.anglePerRotation());
    if (Math.abs(this.stick.getY()) > 0.5) {
      shooterAngleSubsystem.setMotor(this.stick.getY());
    } else {
      shooterAngleSubsystem.setMotor(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterAngleSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
