package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheForce_Shooter extends SubsystemBase {

  private static final int TOP_ID = 12;
  private static final int BOTTOM_ID = 13;

  private static final double SHOOT_VOLTS = 10.0;

  private final TalonFX topMotor = new TalonFX(TOP_ID);
  private final TalonFX bottomMotor = new TalonFX(BOTTOM_ID);

  private final VoltageOut voltageCtrl = new VoltageOut(0);

 private void setShooterVoltage(double volts) {
    topMotor.setControl(voltageCtrl.withOutput(volts));
    bottomMotor.setControl(voltageCtrl.withOutput(volts));
  }

  public Command shootCmd() {
    return startEnd(
        () -> setShooterVoltage(SHOOT_VOLTS), 
        () -> setShooterVoltage(3)            
    );
  }

  
  public Command stopCmd() {
    return runOnce(() -> setShooterVoltage(0));
  }

  
  private final DCMotorSim topSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
          DCMotor.getKrakenX60Foc(1));

  private final DCMotorSim bottomSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
          DCMotor.getKrakenX60Foc(1));

  @Override
  public void simulationPeriodic() {
    topSim.update(0.02);
    bottomSim.update(0.02);
  }
}
