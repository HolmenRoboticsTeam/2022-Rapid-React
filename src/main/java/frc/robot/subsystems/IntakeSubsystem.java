package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kLeftMotorPort);

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void setPosition(boolean open) {
      if (open) {
          intakeMotor.set(IntakeConstants.kSpeed);
      } else {
          intakeMotor.set(0);
      }
  }
}