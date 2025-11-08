// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDriveSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TankDriveCommand extends Command {
  /** Creates a new TankDriveCommand. */
  private TankDriveSubSystem tankDriveSubSystem;
  private DoubleSupplier leftPercent;
  private DoubleSupplier rightPercent;


  public TankDriveCommand(TankDriveSubSystem tankDrive, DoubleSupplier leftPercent, DoubleSupplier rightPercent) {
    tankDriveSubSystem = tankDrive;
    this.leftPercent = leftPercent;
    this.rightPercent = rightPercent;

    addRequirements(tankDriveSubSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPercentDeadbanded = MathUtil.applyDeadband(leftPercent.getAsDouble(), 0.07);
    double rightPercentDeadbanded = MathUtil.applyDeadband(rightPercent.getAsDouble(), 0.07);
    tankDriveSubSystem.drive(leftPercentDeadbanded, rightPercentDeadbanded);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
