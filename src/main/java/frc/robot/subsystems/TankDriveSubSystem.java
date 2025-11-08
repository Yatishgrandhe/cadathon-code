// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDriveSubSystem extends SubsystemBase {
  /** Creates a new TankDriveSubSystem. */
  private TalonFX leftPrimary = new TalonFX(DriveConstants.leftPrimaryID);
  private TalonFX leftFollower = new TalonFX(DriveConstants.leftFollowerID);
  private TalonFX rightPrimary = new TalonFX(DriveConstants.rightPrimaryID);
  private TalonFX rightFollower = new TalonFX(DriveConstants.rightFollowerID);
  public TankDriveSubSystem() {
    leftPrimary.getConfigurator().apply(DriveConstants.configs);
    leftFollower.getConfigurator().apply(DriveConstants.configs);
    rightPrimary.getConfigurator().apply(DriveConstants.configs);
    rightFollower.getConfigurator().apply(DriveConstants.configs);  

    applyCurrentLimits();
    rightPrimary.setInverted(true);
    
    leftFollower.setControl(new Follower(DriveConstants.leftPrimaryID, false));
    rightFollower.setControl(new Follower(DriveConstants.rightPrimaryID, false));
  }

  public void applyCurrentLimits() {
    CurrentLimitsConfigs currentLimitsConfigs = DriveConstants.currentLimitsConfigs;
    leftPrimary.getConfigurator().refresh(currentLimitsConfigs);
    leftFollower.getConfigurator().refresh(currentLimitsConfigs);
    rightPrimary.getConfigurator().refresh(currentLimitsConfigs);
    rightFollower.getConfigurator().refresh(currentLimitsConfigs);

    leftPrimary.getConfigurator().apply(currentLimitsConfigs);
    leftFollower.getConfigurator().apply(currentLimitsConfigs);
    rightPrimary.getConfigurator().apply(currentLimitsConfigs);
    rightFollower.getConfigurator().apply(currentLimitsConfigs);
  }


  public void drive(double leftVelocity, double rightVelocity){
    leftPrimary.set(leftVelocity);
    rightPrimary.set(rightVelocity);
  }
  public void stop(){
    leftPrimary.stopMotor();
    rightPrimary.stopMotor();
  }


  public double getPercentOutLeft() {
    return leftPrimary.get();
  }

  public double getPercentOutRight() {
    return rightPrimary.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Percent Out", getPercentOutLeft());  
    SmartDashboard.putNumber("Right Percent Out", getPercentOutRight());
  }
}
