// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final Spark driveLeftMotor = new Spark(0);
  private final Spark driveRightMotor = new Spark(1);
  private final Encoder leftEncoder = new Encoder(0,1);
  private final Encoder rightEncoder = new Encoder(2,3);
  private final double kEncoderTick2Meter = 1.0/4096*0.128*Math.PI;

  public double getEncoderMeters(){
    return (leftEncoder.get() + rightEncoder.get()) / 2.0 * kEncoderTick2Meter;
  }
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    driveLeftMotor.setInverted(false);
    driveRightMotor.setInverted(true);
    leftEncoder.setReverseDirection(false);
    rightEncoder.setReverseDirection(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Driver encoder value", getEncoderMeters());
    // This method will be called once per scheduler run
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
    rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));
    
    driveLeftMotor.set(leftSpeed);
    driveRightMotor.set(rightSpeed);
  }

  public void stopMotors() {
    setMotors(0, 0);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
