package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManagementConstants;

public class ManagementSubsystem extends SubsystemBase {

  private WPI_VictorSPX managementMotor = new WPI_VictorSPX(ManagementConstants.kManagementMotorPort);

  public ManagementSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Management Speed", managementMotor.get());
  }

  public void setMotor(double speed){
    managementMotor.set(speed);
  }
}
