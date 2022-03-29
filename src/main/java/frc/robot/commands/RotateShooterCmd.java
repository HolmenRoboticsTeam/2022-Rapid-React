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

public class RotateShooterCmd extends CommandBase {
  private final ShooterRotationSubsystem shooterRotationSubsystem;
  private final LimeLightSubsystem limeLight;
  private PIDController pid = new PIDController(0.04, 0, 0.0003);
  private boolean spinLock = false;  // When true, the turret is turning 180 degrees; do not allow other PID input to override it's operation

  /** Creates a new RotateShooterCmd. */
  public RotateShooterCmd(ShooterRotationSubsystem shooterRotationSubsystem, LimeLightSubsystem limeLight) {

    this.shooterRotationSubsystem = shooterRotationSubsystem;
    this.limeLight = limeLight;
    pid.disableContinuousInput();

    SmartDashboard.putNumber("Shooter Minimum Speed", ShooterConstants.kMinimumRotationSpeed);
    SmartDashboard.putNumber("Shooter Maximum Speed", ShooterConstants.kMaximumRotationSpeed);
    SmartDashboard.putNumber("Shooter Rotation Slow Down Angle", ShooterConstants.rotationConstantAngle);

    addRequirements(shooterRotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter Rotation X-Angle Offset", 0);
    SmartDashboard.putNumber("Shooter Rotation PID Speed", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (this.limeLight.hasTarget() == (Number)1.0) {
      double tX = this.limeLight.getDoubleTX();

      double speed = pid.calculate(tX, 0);
      speed = MathUtil.clamp(speed, -0.3, 0.3); // create constant for it soon
      SmartDashboard.putNumber("Shooter Rotation X-Angle Offset", tX);
      SmartDashboard.putNumber("Shooter Rotation PID Speed", speed);
      SmartDashboard.putNumber("Distance to Target Meters", this.limeLight.getDistanceToTargetMeters());

      shooterRotationSubsystem.setMotor(speed); // TODO: replace with speed
      if (!spinLock) {
        // Logic to determine if to spin the turret
        // This should be based on the angle of the gyro
        // When the angle of gyro (robot's rotation) exceeds 180 degrees, the turret should spin to the opposite bound
      }
    //}
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
