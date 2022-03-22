// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCmd extends ParallelCommandGroup {   // button type parallels with the management system

  private ShooterSubsystem shooterSubsystem;
  // private PIDController pidController;
  // private double targetSpeed;

  public ShooterCmd(ShooterSubsystem shooterSubsystem) {

    // this.shooterSubsystem = shooterSubsystem;
    // this.pidController = new PIDController(1.0, 0.0, 0.0);
    // this.targetSpeed = targetSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // this.pidController.reset();
    // this.pidController.setSetpoint(this.targetSpeed);

  } //

  @Override
  public void execute() {
    // double currentSpeed = this.shooterSubsystem.getSpeedMeters(); // recalculate uhhhhyeah
    // double speed = this.pidController.calculate(currentSpeed);
    // speed = MathUtil.clamp(speed, -0.65, 0.65);  // might be changed
    // System.out.println(speed);

    // this.shooterSubsystem.setMotor(speed);
    shooterSubsystem.setMotor(ShooterConstants.shooterRunSpeed);
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
