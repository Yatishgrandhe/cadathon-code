// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class TheForce_Hanging extends SubsystemBase { 
    private static final int VERTICAL_MOTOR_ID = 10;
    private static final int SIDEWAYS_MOTOR_ID = 11;
    
    private static final double VERTICAL_VOLTS = 3.0;
    public static final double SIDEWAYS_VOLTS = 3.0;

    private final TalonFX verticalMotor = new TalonFX(VERTICAL_MOTOR_ID);
    private final TalonFX sidewaysMotor = new TalonFX(SIDEWAYS_MOTOR_ID);
    
    private static final int HOOK_SOLENOID_FORWARD_CHANNEL = 0;
    private static final int HOOK_SOLENOID_REVERSE_CHANNEL = 1;
    private final DoubleSolenoid hookSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        HOOK_SOLENOID_FORWARD_CHANNEL,
        HOOK_SOLENOID_REVERSE_CHANNEL
    );

    public void setVerticalMotorPower(double volts) {
        verticalMotor.setControl(new VoltageOut(volts));
    }

    public void setSidewaysMotorPower(double volts) {
        sidewaysMotor.setControl(new VoltageOut(volts));
    }

    public void hook() {
        hookSolenoid.set(Value.kForward);
    }

    public void unhook() {
        hookSolenoid.set(Value.kReverse);
    }

    public void setMotorPower(double volts) {
        setVerticalMotorPower(volts);
    }

    public Command moveHangingUp() {
        return run(() -> setVerticalMotorPower(VERTICAL_VOLTS))
            .finallyDo(() -> setVerticalMotorPower(0));
    }
    
    public Command moveHangingDown() {
        return run(() -> setVerticalMotorPower(-VERTICAL_VOLTS))
            .finallyDo(() -> setVerticalMotorPower(0));
    }
}
