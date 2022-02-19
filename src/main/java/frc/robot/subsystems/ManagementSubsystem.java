package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManagementConstants;

public class ManagementSubsystem extends SubsystemBase {

  private WPI_VictorSPX managementMotor = new WPI_VictorSPX(ManagementConstants.kManagementMotorPort);

  public ManagementSubsystem() {}

  @Override
  public void periodic() {} //

  public void toggleManagement(boolean open){
    if (open) {
      managementMotor.set(ManagementConstants.kSpeed);
    }else{
      managementMotor.set(0);
    }

  }
}
