package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;

public class ColorWheelSubsystem extends SubsystemBase{

    private static final int COLOR_WHEEL_MOTOR_ID = 12;
    private static final double SPIN_SPEED = 0.25;

    private final TalonFX wheelMotor = new TalonFX(COLOR_WHEEL_MOTOR_ID);

    private DetectedColor lastColor = DetectedColor.UNKNOWN;
    private int colorTransitions = 0;
    private boolean isSpinning = false;
    private DetectedColor lastStableColor = DetectedColor.UNKNOWN;
    private int sameColorCount = 0;
    private static final int DEBOUNCE_COUNT = 3;
    
    public void spinWheel(double speed) {
        wheelMotor.setControl(new VoltageOut(speed * 12.0));
        isSpinning = true;
    }

    public void stopWheel() {
        wheelMotor.setControl(new VoltageOut(0.0));
        isSpinning = false;
    }
    public DetectedColor getDebouncedColor() {
        DetectedColor current = getCurrentColor();
        
        if (current == lastStableColor) {
            sameColorCount++;
        } else {
            sameColorCount = 0;
        }
        
        if (sameColorCount >= DEBOUNCE_COUNT) {
            return current;
        }
        
        return lastStableColor;
    }
    public DetectedColor getCurrentColor() {
        return DetectedColor.UNKNOWN;
    }

    public void updateColorCounter() {
        if (!isSpinning) {
            return;
        }
        DetectedColor currentColor = getCurrentColor();
        if (currentColor != DetectedColor.UNKNOWN && currentColor != lastColor) {
            colorTransitions++;
            lastColor = currentColor;
        }
    }
    public double getRotations() {
        return colorTransitions / 8.0;
    }
    public void resetRotationCounter() {
        colorTransitions = 0;
        lastColor = DetectedColor.UNKNOWN;
    }
    private static final Map<DetectedColor, DetectedColor> fmsToSensorMap = Map.of(
        DetectedColor.RED, DetectedColor.RED,
        DetectedColor.BLUE, DetectedColor.BLUE,
        DetectedColor.YELLOW, DetectedColor.YELLOW,
        DetectedColor.GREEN, DetectedColor.GREEN
    );
    public DetectedColor getTargetSensorColor(DetectedColor fmsColor) {
        return fmsToSensorMap.get(fmsColor);
    }
    @Override
    public void periodic() {
        updateColorCounter();
        SmartDashboard.putString("Color Wheel/Current Color", getCurrentColor().toString());
        SmartDashboard.putNumber("Color Wheel/Transitions", colorTransitions);
        SmartDashboard.putNumber("Color Wheel/Rotations", getRotations());
    }
}
