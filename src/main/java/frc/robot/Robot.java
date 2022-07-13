// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

  public static CANSparkMax motor1;
  public static CANSparkMax motor2;
  public static CANSparkMax motor3;
  public static CANSparkMax motor4;
  public static CANSparkMax motor5;
  public static CANSparkMax motor6;

  public static MotorControllerGroup leftMotors;
  public static MotorControllerGroup rightMotors;

  public static DifferentialDriveOdometry odometer;
  public static AHRS gyro;
  public static ColorSensorV3 sensor;
  public static I2C.Port I2C;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");

  @Override
  public void robotInit() {
    motor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor3 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor4 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor5 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor6 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotors = new MotorControllerGroup(motor1, motor2, motor3);
    rightMotors = new MotorControllerGroup(motor4, motor5, motor6);
    gyro = new AHRS();
    odometer = new DifferentialDriveOdometry(gyro.getRotation2d());
    gyro.reset();
    motor1.getEncoder().setPosition(0.0);
    sensor = new ColorSensorV3(Port.kOnboard);

  }

  @Override
  public void robotPeriodic() {
    moveDistance(2.71818);
    turnToAngle(180);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
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
    return sensor.getProximity();
  }

  /**
   * Determines which color is closest to the color detected by the sensor
   * 
   * @return
   * 
   * @return The color that is closest to what the color sensor detects (RED,
   *         BLUE, OTHER or NONE)
   */
  public ColorChoices getDetectedColor() {
    int red = sensor.getRed();
    int blue = sensor.getBlue();
    if (red > blue)
      return ColorChoices.RED;
    else if (blue > red)
      return ColorChoices.BLUE;
    else
      return ColorChoices.NONE;
  }

  /**
   * Logs important color sensor values to SmartDashboard
   * 
   * @param proximity How close the nearest object is to the sensor
   * @param color     What color is detected by the sensor
   */
  public void logToDashboard2(int proximity, ColorChoices color) {
    // TODO write method
    SmartDashboard.putNumber("Proxmity", sensor.getProximity());
    SmartDashboard.putString("Color", getDetectedColor().toString());
  }

  /* LIMELIGHT STATION */

  /**
   * Determines whether or not there is a target (as determined by limelight)
   * 
   * @return whether or not a target is visible in the frame
   */
  public boolean existsTarget() {
    return tv.getBoolean(false);
  }

  /**
   * Determines horizontal offset from center of the target (as calculated by
   * limelight)
   * 
   * @return the horizontal offset to the center of the target
   */
  public double getHorizontalOffset() {
    double xoffset = tx.getDouble(0.0);
    return xoffset;
  }

  /**
   * Determines whether or not the robot is aligned with the goal
   * 
   * @return if the robot is aligned with the goal
   */
  public boolean isAligned() {
    if (getHorizontalOffset() == 0)
      return true;
    else
      return false;
  }

  /**
   * Aligns the robot with a target
   * 
   * @param horizontalError the horizontal offset (as reported by limelight)
   */
  public void alignRobot(double horizontalError) {
    if (getHorizontalOffset() < 0) {
      leftMotors.set(-0.1);
      rightMotors.set(-0.1);
    } else if (getHorizontalOffset() > 0) {
      leftMotors.set(0.1);
      rightMotors.set(0.1);
    } else {
      leftMotors.set(0);
      rightMotors.set(0);
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
    double currRotat = getHeading();
    if (degrees < 0) {
      if (currRotat < degrees) {
        leftMotors.set(0.15);
        rightMotors.set(0.15);
      }
    } else if (degrees > 0) {
      if (currRotat < degrees) {
        leftMotors.set(-0.15);
        rightMotors.set(-0.15);
      }
    } else {
      leftMotors.set(0);
      rightMotors.set(0);
    }
  }

  /**
   * Moves the robot a specified distance forwards or backwards
   * 
   * @param distance distance in meters (can be negative to move the robot
   *                 backwards)
   */
  public void moveDistance(double distance) {
    double currPosit = motor1.getEncoder().getPosition();
    if (distance < 0) {
      if (currPosit < distance) {
        leftMotors.set(0.15);
        rightMotors.set(-0.15);
      }
    } else if (distance > 0) {
      if (currPosit < distance) {
        leftMotors.set(-0.15);
        rightMotors.set(0.15);
      }
    } else {
      leftMotors.set(0);
      rightMotors.set(0);
    }

  }

  /**
   * Updates the odometry with distance measurements from encoders
   * 
   */
  public void updateOdometry() {
    double metersMoved = 0.1524 * Math.PI / 10.81;
    double positionR = motor4.getEncoder().getPosition();
    double positionL = motor1.getEncoder().getPosition();
    odometer.update(gyro.getRotation2d(), positionL * metersMoved, positionR * metersMoved);
  }

  /**
   * Gets the distance of the robot from its starting point
   * 
   * @return distance in meters
   */
  public double getDistanceTraveled() {
    double x = odometer.getPoseMeters().getX();
    double y = odometer.getPoseMeters().getY();
    double hypotenuseSquared = Math.pow(x, 2) + Math.pow(y, 2);
    double hypotenuse = Math.pow(hypotenuseSquared, 0.5);
    return hypotenuse;
  }

  /**
   * Logs important position values to SmartDashboard
   */
  public void logToDashboard(double xPos, double yPos, double rotation) {
    SmartDashboard.putNumber("x Positon", odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("y Positon", odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("Rotation", gyro.getAngle());
    SmartDashboard.putNumber("Left Encoder", motor1.getEncoder().getPosition());
  }
}
