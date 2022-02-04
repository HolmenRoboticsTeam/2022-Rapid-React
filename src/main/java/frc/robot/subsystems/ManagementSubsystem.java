package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.magamentconstants;

public class ManagementSubsystem extends SubsystemBase {

  private WPI_VictorSPX managenermoter = new WPI_VictorSPX(magamentconstants.magamentoterID);

  public ManagementSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggelmagement(boolean toggel){
    if (toggel) {
      managenermoter.set(magamentconstants.managerrunspeed);
    }else{
      managenermoter.set(magamentconstants.magegerfullstop);
    }

  }
}
