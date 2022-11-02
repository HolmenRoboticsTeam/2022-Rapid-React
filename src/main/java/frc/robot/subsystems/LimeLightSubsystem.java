// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterAngleConstants;

public class LimelightSubsystem extends SubsystemBase {
  // Network tables
  private NetworkTable limelightTable;

  // Network table entries
  private NetworkTableEntry xOffsetEntry;
  private NetworkTableEntry yOffsetEntry;
  private NetworkTableEntry areaEntry;
  private NetworkTableEntry targetFoundEntry;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    // Network tables
    this.limelightTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kNetworkTable);

    // Network table entries
    this.xOffsetEntry = this.limelightTable.getEntry(LimelightConstants.kXOffsetEntry);
    this.yOffsetEntry = this.limelightTable.getEntry(LimelightConstants.kYOffsetEntry);
    this.areaEntry = this.limelightTable.getEntry(LimelightConstants.kTargetAreaEntry);
    this.targetFoundEntry = this.limelightTable.getEntry(LimelightConstants.kTargetFoundEntry);

    // Telemetry
    SmartDashboard.putBoolean("Limelight Has Target", false);
    SmartDashboard.putNumber("Target Distance", 0);
    SmartDashboard.putNumber("Required Angle", 0);
    SmartDashboard.putNumber("Required Velocity", 0);
  }

  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putNumber("Target Distance", this.getTargetDistanceSimple());
    SmartDashboard.putBoolean("Limelight Has Target", this.getTargetFound());
    SmartDashboard.putNumber("Required Angle", Units.radiansToDegrees(this.getShooterAngleRequired()));
    SmartDashboard.putNumber("Required Velocity", this.getShooterVelocityRequired());
  }

  /**
   * Get the x-offset of any identified targets in degrees.
   * @return the x-offset of a target in degrees.
   */
  public double getXOffsetDegrees() {
    return this.xOffsetEntry.getDouble(0.0);
  }

  /**
   * Get the y-offset of any identified targets in degrees.
   * @return the y-offset of a target in degrees.
   */
  public double getYOffsetDegrees() {
    return this.yOffsetEntry.getDouble(0.0);
  }

  /**
   * Get the area of a target in raw pixels.
   * @return the area of a target in raw pixels.
   */
  public double getAreaPixels() {
    return this.areaEntry.getDouble(0.0);
  }

  /**
   * Get if a target has been identified by the limelight.
   * @return if a target is identified
   */
  public boolean getTargetFound() {
    double hasTarget = this.targetFoundEntry.getDouble(0.0);
    return hasTarget > 0;
  }

  /**
   * Get the distance to the target without accounting for camera warp.
   * @return The distance to the target.
   */
  public double getTargetDistanceSimple() {
    double yOffsetDegrees = this.getYOffsetDegrees();
    double distance = LimelightConstants.kHeightOfTargetFromLimelightMeters / Math.tan(
      Math.toRadians(yOffsetDegrees + LimelightConstants.kLimelightMountAngle)
    );
    return distance;
  }

  /**
   * Calculate the required angle of the shooter using the distance to the target.
   * @return The angle required in radians.
   */
  public double getShooterAngleRequired() {
    // Taken from: https://www.desmos.com/calculator/lazkfsymei
    // angleRequired = atan(tan(S) * d - 2H
    //                      _______________
    //                            -d       )
    // Where:
    //        S = entry angle
    //        d = distance to target
    //        H = height of target from limelight
    //
    // All values in radians and meters
    double S = ShooterAngleConstants.kEntryAngleRadians;
    double d = this.getTargetDistanceSimple();
    double H = LimelightConstants.kHeightOfTargetFromLimelightMeters;
    return Math.atan(
      (Math.tan(S) * d - (2.0 * H)) / -d
    );
  }

  /**
   * Calculate the required velocity of the shooter suing the distance to the target.
   * @return The velocity required in m/s.
   */
  public double getShooterVelocityRequired() {
    // Taken from: https://www.desmos.com/calculator/lazkfsymei
    // velocityRequired = sqrt(-9.8 * d^2 * (1 + tan(a)^2)
    //                         ___________________________
    //                               2H - 2d * tan(a)     )
    // Where:
    //        a = angle required
    //        d = distance to target
    //        H = height of target from limelight
    //
    // All values in radians and meters
    double a = this.getShooterAngleRequired();
    double d = this.getTargetDistanceSimple();
    double H = LimelightConstants.kHeightOfTargetFromLimelightMeters;
    return Math.sqrt(
      (-9.8 * Math.pow(d, 2) * (1.0 + Math.pow(Math.tan(a), 2)))
      / (2.0 * H - 2.0 * d * Math.tan(a))
    );
  }
}
