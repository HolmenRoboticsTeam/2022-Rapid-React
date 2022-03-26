package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kLeftMotorPort);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
  }

  public void setMotor(double speed) {
    intakeMotor.set(0);
  }
}