// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}



  /* COLOR SENSOR STATION */

  public enum ColorChoices {
    RED, BLUE, OTHER, NONE
  }

  /**
   * Proximity to the closest item in front of sensor
   * @return proximity to the closest ball object in front of the sensor
   */
  public int getProximity() { return -1; } 

  /**
   * Determines which color is closest to the color detected by the sensor
   * @return The color that is closest to what the color sensor detects (RED, BLUE, OTHER or NONE)
   */
  public ColorChoices getDetectedColor() { return null; }

  /**
   * Logs values to SmartDashboard
   * @param proximity How close the nearest object is to the sensor
   * @param color What color is detected by the sensor
   */
  public void logToDashboard(int proximity, ColorChoices color) {}



  /* LIMELIGHT STATION */

  /**
   * Determines whether or not there is a target (as determined by limelight)
   * @return whether or not a target is visible in the frame
   */
  public boolean existsTarget() { return false; }

  /**
   * Determines horizontal offset from center of the target (as calculated by limelight)
   * @return the horizontal offset to the center of the target
   */
  public double getHorizontalOffset() { return -1.0; }

  /**
   * Determines whether or not the robot is aligned with the goal
   * @return if the robot is aligned with the goal
   */
  public boolean isAligned() { return false; }

  /**
   * Aligns the robot with a target
   * @param horizontalError the horizontal offset (as reported by limelight)
   */
  public void alignRobot(double horizontalError) {}

  /**
   * Logs important Limelight values to SmartDashboard
   * @param isAligned whether or not the robot is aligned with the goal
   * @param existsTarget whether or not a target is visible in the frame
   * @param horizontalError the horizontal offset to the center of the goal
   */
  public void logToDashboard(boolean isAligned, boolean existsTarget, double horizontalError) {} // TODO: log stuff


}
