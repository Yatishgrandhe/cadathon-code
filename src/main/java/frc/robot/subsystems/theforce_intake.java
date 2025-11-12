package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class theforce_intake extends SubsystemBase {
  private static final int INTAKE_ID = 9;

  private static final double INTAKE_IN_VOLTS = 3.0;

  private final TalonFX motor = new TalonFX(INTAKE_ID);

  private void setMotorPower(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  public Command moveIntakeIn() {
    return startEnd(() -> setMotorPower(INTAKE_IN_VOLTS), () -> setMotorPower(0));
  }
  
}
