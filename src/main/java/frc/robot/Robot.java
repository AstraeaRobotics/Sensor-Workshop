// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import com.revrobotics.ColorSensorV3;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {

  I2C.Port i2cPort = I2C.Port.kOnboard;

  ColorSensorV3 colorSensor;

  NetworkTableInstance instance;
  NetworkTable colorTable;
  NetworkTable limelightTable;

  CANSparkMax leftMotor1;
  CANSparkMax leftMotor2;
  CANSparkMax leftMotor3;
  CANSparkMax rightMotor1;
  CANSparkMax rightMotor2;
  CANSparkMax rightMotor3;

  PS4Controller joystick;

  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;

  AHRS gyro;

  @Override
  public void robotInit() {
    colorSensor = new ColorSensorV3(i2cPort);
    colorTable = instance.getDefault().getTable("FMSInfo");
    limelightTable = instance.getDefault().getTable("limelight");

    leftMotor1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotor2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotor3 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotor3 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    joystick = new PS4Controller(0);

    leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
    rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

    gyro = new AHRS();
  }

  @Override
  public void robotPeriodic() {
    logToDashboard(getProximity(), getDetectedColor());
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if (joystick.getCrossButton()) {
      alignRobot(getHorizontalOffset(), getVerticalOffset());
    } else {
      leftMotors.set(0);
      rightMotors.set(0);
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
    return colorSensor.getProximity();
  }

  /**
   * Determines which color is closest to the color detected by the sensor
   * 
   * @return The color that is closest to what the color sensor detects (RED,
   *         BLUE, OTHER or NONE)
   */
  public ColorChoices getDetectedColor() {
    // TODO write method
    if (getProximity() < 150) {
      return ColorChoices.NONE;
    }
    int blueColorValue = colorSensor.getBlue();
    int redColorValue = colorSensor.getRed();
    if (blueColorValue > redColorValue) {
      return ColorChoices.BLUE;
    } else {
      return ColorChoices.RED;
    }
  }

  /**
   * Logs important color sensor values to SmartDashboard
   * 
   * @param proximity How close the nearest object is to the sensor
   * @param color     What color is detected by the sensor
   */
  public void logToDashboard(int proximity, ColorChoices color) {
    // TODO write method
    SmartDashboard.putString("Color", color.toString());
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putBoolean("Right Color", isRightColor());
    // ShuffleboardTab colorTab = Shuffleboard.getTab("Color");
    // colorTab.add("Color", color.toString());
    // colorTab.add("Proximity", proximity);
  }

  public boolean isRightColor() {
    if (colorTable.getEntry("isRedAlliance").getBoolean(false)) {
      return getDetectedColor() == ColorChoices.BLUE;
    } else {
      return getDetectedColor() == ColorChoices.RED;
    }
  }

  /* LIMELIGHT STATION */

  /**
   * Determines whether or not there is a target (as determined by limelight)
   * 
   * @return whether or not a target is visible in the frame
   */
  public boolean existsTarget() {
    // TODO write method

    return limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  /**
   * Determines horizontal offset from center of the target (as calculated by
   * limelight)
   * 
   * @return the horizontal offset to the center of the target
   */
  public double getHorizontalOffset() {
    // TODO write method
    return limelightTable.getEntry("tx").getDouble(0);
  }

  public double getVerticalOffset() {
    // TODO write method
    return limelightTable.getEntry("ty").getDouble(0);
  }

  /**
   * Determines whether or not the robot is aligned with the goal
   * 
   * @return if the robot is aligned with the goal
   */
  public boolean isAligned() {
    return (Math.abs(getHorizontalOffset()) <= 5) && (Math.abs(getVerticalOffset()) <= 5);
  }

  /**
   * Aligns the robot with a target
   * 
   * @param horizontalError the horizontal offset (as reported by limelight)
   */
  public void alignRobot(double horizontalError, double verticalError) {
    // TODO write method
    if (existsTarget()) {
      if (!isAligned()) {
        double rotConstant = 0.007;
        leftMotors.set((horizontalError + verticalError) * rotConstant);
        rightMotors.set((horizontalError - verticalError) * rotConstant);
      } else {
        leftMotors.set(0);
        rightMotors.set(0);
      }
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
    return gyro.getAngle();
  }

  /**
   * Turns the robot to a specified angle
   * 
   * @param degrees angle in degrees
   */
  public void turnToAngle(double degrees) {
    leftMotors.set(degrees * 0.01);
    rightMotors.set(degrees * 0.01);
  }

  /**
   * Moves the robot a specified distance forwards or backwards
   * 
   * @param distance distance in meters (can be negative to move the robot
   *                 backwards)
   */
  public void moveDistance(double distance) {
    double currentRevs = leftMotor1.getEncoder().getPosition();
    double targetRevs = ((distance * 12 * 10.75) / (6 * Math.PI));

    if (currentRevs >= targetRevs) {
      leftMotors.set(0);
      rightMotors.set(0);
    } else {
      leftMotors.set(0.05);
      rightMotors.set(-0.05);
    }
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
