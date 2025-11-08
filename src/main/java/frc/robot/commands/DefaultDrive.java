// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DefaultDrive extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final CommandJoystick m_leftJoystick;
  private final CommandJoystick m_rightJoystick;

  public DefaultDrive(DriveSubsystem driveSubsystem, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    m_driveSubsystem = driveSubsystem;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    // Tank drive - left joystick controls left motor, right joystick controls right motor
    double leftSpeed = -m_leftJoystick.getY();
    double rightSpeed = -m_rightJoystick.getY();
    
    // Deadband
    if (Math.abs(leftSpeed) < 0.05) {
      leftSpeed = 0;
    }
    if (Math.abs(rightSpeed) < 0.05) {
      rightSpeed = 0;
    }
    
    m_driveSubsystem.setMotors(leftSpeed, rightSpeed);
    
    // Debug info
    SmartDashboard.putNumber("Left Joystick Y", m_leftJoystick.getY());
    SmartDashboard.putNumber("Right Joystick Y", m_rightJoystick.getY());
    SmartDashboard.putNumber("Left Motor Speed", leftSpeed);
    SmartDashboard.putNumber("Right Motor Speed", rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

