// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.KebabCaseStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tx");

  CANSparkMax rightMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFollow1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFollow2 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftMaster = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftFollow1 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftFollow2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftFollow1, leftFollow2);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightFollow1, rightFollow2);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (!isAligned()) {
      alignRobot(getHorizontalOffset());
    } else {
      rightMotors.set(0);
      leftMotors.set(0);
    }
  }

  /* COLOR SENSOR STATION */

  public enum ColorChoices {
    RED, BLUE, OTHER, NONE
  }

  /**
   * Proximity to the closest item in front of sensor
   * 
   * @return proximity to the closest ball object in front of the sensor
   */
  public int getProximity() {
    // TODO write method
    return -1;
  }

  /**
   * Determines which color is closest to the color detected by the sensor
   * 
   * @return The color that is closest to what the color sensor detects (RED,
   *         BLUE, OTHER or NONE)
   */
  public ColorChoices getDetectedColor() {
    // TODO write method
    return null;
  }

  /**
   * Logs important color sensor values to SmartDashboard
   * 
   * @param proximity How close the nearest object is to the sensor
   * @param color     What color is detected by the sensor
   */
  public void logToDashboard(int proximity, ColorChoices color) {
    // TODO write method
  }

  /* LIMELIGHT STATION */

  /**
   * Determines whether or not there is a target (as determined by limelight)
   * 
   * @return whether or not a target is visible in the frame
   */
  public boolean existsTarget() {
    // TODO write method
    boolean v = tv.getBoolean(false);
    return v;
  }

  /**
   * Determines horizontal offset from center of the target (as calculated by
   * limelight)
   * 
   * @return the horizontal offset to the center of the target
   */
  public double getHorizontalOffset() {
    // TODO write method
    double x = tx.getDouble(0.0);
    return x;
  }

  /**
   * Determines whether or not the robot is aligned with the goal
   * 
   * @return if the robot is aligned with the goal
   */
  public boolean isAligned() {
    // TODO write method
    double x = getHorizontalOffset();
    if (Math.abs(x) <= 2)
    {
      return true;
    }
    return false;
  }

  /**
   * Aligns the robot with a target
   * 
   * @param horizontalError the horizontal offset (as reported by limelight)
   */
  public void alignRobot(double horizontalError) {
    if (horizontalError < 0) {
      rightMotors.set(-.1);
      leftMotors.set(-.1);
    } else if (horizontalError > 0) {
      rightMotors.set(0.1);
      leftMotors.set(0.1);
    }
  }

  /**
   * Logs important Limelight values to SmartDashboard
   * 
   * @param isAligned       whether or not the robot is aligned with the goal
   * @param existsTarget    whether or not a target is visible in the frame
   * @param horizontalError the horizontal offset to the center of the goal
   */
  public void logToDashboard(boolean isAligned, boolean existsTarget, double horizontalError) {
    // TODO write method
  }

  /* GYRO + ENCODER STATION */

  /**
   * Returns the angle that the robot is facing, relative to it's initial heading
   * (when it was turned on)
   * 
   * @return the yaw angle in degrees
   */
  public double getHeading() {
    // TODO write method
    return 0.0;
  }

  /**
   * Turns the robot to a specified angle
   * 
   * @param degrees angle in degrees
   */
  public void turnToAngle(double degrees) {
    // TODO write method
  }

  /**
   * Moves the robot a specified distance forwards or backwards
   * 
   * @param distance distance in meters (can be negative to move the robot
   *                 backwards)
   */
  public void moveDistance(double distance) {
    // TODO write method
  }

  /**
   * Updates the odometry with distance measurements from encoders
   * 
   */
  public void updateOdometry() {
    // TODO write method
  }

  /**
   * Gets the distance of the robot from its starting point
   * 
   * @return distance in meters
   */
  public double getDistanceTraveled() {
    // TODO write method
    return 0.0;
  }

  /**
   * Logs important position values to SmartDashboard
   */
  public void logToDashboard(double xPos, double yPos, double rotation) {
    // TODO write method
  }
}
