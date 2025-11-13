// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/** Intialize motor and set motor power for hanging mechanism*/
public class TheForce_Hanging extends SubsystemBase { 
    private static final int INTAKE_ID = 10;

    private static final double INTAKE_IN_VOLTS = 3.0;

    private final TalonFX motor = new TalonFX(INTAKE_ID);

    private void setMotorPower(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    public Command moveHangingUp() {
        return run(() -> setMotorPower(INTAKE_IN_VOLTS))
            .finallyDo(() -> setMotorPower(0));
    }
    
    public Command moveHangingDown() {
        return run(() -> setMotorPower(-INTAKE_IN_VOLTS))
            .finallyDo(() -> setMotorPower(0));
    }
}