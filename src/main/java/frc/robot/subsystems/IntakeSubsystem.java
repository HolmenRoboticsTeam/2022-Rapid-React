package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(3);

    public IntakeSubsystem() {}

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void setPosition(boolean open) {
      if (open) {
          intakeMotor.set(-1);
      } else {
          intakeMotor.set(1);
      }
  }
    
}