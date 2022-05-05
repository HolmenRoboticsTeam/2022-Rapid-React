// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  public boolean hasTarget() {
    return tv.getNumber(0).doubleValue() == 1.0;
  }

  @Override
  public void periodic() {} //
}
