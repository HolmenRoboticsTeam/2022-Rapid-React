package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManagementConstants;

public class ManagementSubsystem extends SubsystemBase {

  private WPI_VictorSPX managenermoter = new WPI_VictorSPX(ManagementConstants.kManagementMotorPort);

  public ManagementSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggelmagement(boolean open){
    if (open) {
      managenermoter.set(ManagementConstants.kSpeed);
    }else{
      managenermoter.set(0);
    }

  }
}
