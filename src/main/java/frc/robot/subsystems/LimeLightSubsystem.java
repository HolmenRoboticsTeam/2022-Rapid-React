// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta; // target area
  private final NetworkTableEntry tv; // target valid

  /** Creates a new LimeLightSubsystem. */
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

  public double getDoubleTV() {
    return (tv.getDouble(0.0));
  }

  public boolean findTarget() {
    return (this.getDoubleTA() > 0 || this.getDoubleTX() > 0 || this.getDoubleTY() > 0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
