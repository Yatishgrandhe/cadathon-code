package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// It's an entry point required by the Java to initiate the program
public final class Main {
  // Private constructor prevents instantiation since this is a utility class with only static methods
  private Main() {}

  // Main method is required by Java to start the application, and WPILib needs this to initialize the robot framework
  public static void main(String... args) {
    // RobotBase.startRobot is called to properly initialize WPILib's robot lifecycle and start the Robot class
    RobotBase.startRobot(Robot::new);
  }
}
