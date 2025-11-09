package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Extends TimedRobot to get periodic callbacks from WPILib's robot lifecycle management system
public class Robot extends TimedRobot {
  // Stores autonomous command reference so it can be cancelled when switching to teleop mode
  private Command m_autonomousCommand;

  // RobotContainer is created here to initialize all subsystems and commands at robot startup
  private final RobotContainer m_robotContainer;

  public Robot() {
    // RobotContainer must be created in constructor to ensure subsystems are ready before robot starts
    m_robotContainer = new RobotContainer();
  }

  @Override
  // This method runs continuously to execute scheduled commands, which is required for the command-based framework
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  // Empty because no initialization needed when robot becomes disabled
  public void disabledInit() {}

  @Override
  // Empty because no periodic actions needed during disabled state
  public void disabledPeriodic() {}

  @Override
  // Called once when autonomous mode starts to begin the autonomous routine
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Only schedule if autonomous command exists to avoid null pointer exceptions
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  // Empty because command scheduler handles autonomous execution in robotPeriodic
  public void autonomousPeriodic() {}

  @Override
  // Cancels autonomous command when teleop starts to prevent conflicts between modes
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  // Empty because command scheduler handles teleop execution in robotPeriodic
  public void teleopPeriodic() {}

  @Override
  // Cancels all commands in test mode to ensure clean state for testing individual components
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  // Empty because test mode typically uses direct subsystem access rather than commands
  public void testPeriodic() {}

  @Override
  // Empty because simulation initialization is handled elsewhere if needed
  public void simulationInit() {}

  @Override
  // Empty because simulation updates are handled elsewhere if needed
  public void simulationPeriodic() {}
}
