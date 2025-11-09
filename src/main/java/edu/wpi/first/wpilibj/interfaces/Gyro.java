package edu.wpi.first.wpilibj.interfaces;
public interface Gyro {
    /**
     * Reset the gyro to zero heading.
     * This method is called by NavX's reset() method.
     */
    void reset();
    
    /**
     * Get the current angle in degrees.
     * This method is called by NavX's getAngle() method.
     * 
     * @return the current angle in degrees
     */
    double getAngle();
}

