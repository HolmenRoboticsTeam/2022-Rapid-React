// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterRotationSubsystem;

public class AltRotateShooter extends CommandBase {
  private final ShooterRotationSubsystem shooterRotationSubsystem;
  private final Double target;
  private PIDController pid = new PIDController(0.04, 0, 0.0003);

  /** Creates a new RotateShooterCmd. */
  public AltRotateShooter(ShooterRotationSubsystem shooterRotationSubsystem, Double target) {

    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.target = target;
    pid.disableContinuousInput();

    addRequirements(shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tX = shooterRotationSubsystem.getEncoderValue();

    double speed = pid.calculate(tX, target); //Current encoder value, minimum or max bound 
    speed = MathUtil.clamp(speed, -0.35, 0.35);
    shooterRotationSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRotationSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
