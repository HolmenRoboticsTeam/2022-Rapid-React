package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
 // private int kMotor = new int(ElevatorConstants.kMotorPort); // Int is temporary....

    public ClimberSubsystem() {

    }

    @Override
    public void setPosition(boolean open) {
      if (open) {
          intakeMotor.set(IntakeConstants.kSpeed);
      } else {
          intakeMotor.set(0);
      }
  }

}
