// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class LimeLightSubsystem extends SubsystemBase {
  private final NetworkTable table;
  private final NetworkTableEntry tx;  // x-offset in degrees
  private final NetworkTableEntry ty;  // y-offset in degrees
  private final NetworkTableEntry ta;  // area of target
  private final NetworkTableEntry tv;  // boolean, has any target?

  public LimeLightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
  }

  public double getDoubleTX() {
    return (tx.getDouble(0.0));
  }

  public double getDoubleTY() {
    return (ty.getDouble(0.0));
  }

  public double getDoubleTA() {
    return (ta.getDouble(0.0));
  }

  public double getDistanceToTargetMeters() {
    double tY = getDoubleTY();
    double angleToGoalDegrees = (ShooterConstants.kLimelightMountAngle + tY);
    double angleToGoalRadians = (angleToGoalDegrees * (Math.PI / 180.0));
    return ((ShooterConstants.kHeightOfTargetMeters - ShooterConstants.kLimeLightHeightFromGroundMeters) / (Math.tan(angleToGoalRadians)));
  }

  public double shootingAngleNeededInMeters() {
    double targetDistance = getDistanceToTargetMeters();
    // System.out.println(
    //   Math.atan(
    //     (Math.tan(ShooterConstants.kEntryAngle) * targetDistance - (2.0 * ShooterConstants.kHeightOfLimelightFromTarget)) / -targetDistance
    //   )
    // );
    return Math.atan(
      (Math.tan(ShooterConstants.kEntryAngle) * targetDistance - (2.0 * ShooterConstants.kHeightOfLimelightFromTarget)) / -targetDistance
    );
  }

  public double exitVelocityNeededInMeters() {
    double targetDistance = getDistanceToTargetMeters();
    double entryAngle = shootingAngleNeededInMeters();
    return 1.5 * Math.sqrt(
      -(9.8 * Math.pow(targetDistance, 2) * (1 + Math.pow(Math.tan(entryAngle), 2))) / (2.0 * ShooterConstants.kHeightOfLimelightFromTarget - 2.0 * targetDistance * Math.tan(entryAngle))
    );
  }

  public boolean hasTarget() {
    return tv.getNumber(0).doubleValue() == 1.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SHOOTER: VELOCITY NEEDED", exitVelocityNeededInMeters());
    SmartDashboard.putNumber("SHOOTER: SHOOTING ANGLE NEEDED", Math.toDegrees(shootingAngleNeededInMeters()));
  } //
}
