// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.KebabCaseStrategy;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;

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

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSense = new ColorSensorV3(i2cPort);

  RelativeEncoder rightMEnc = rightMaster.getEncoder();
  RelativeEncoder leftMEnc = leftMaster.getEncoder();
  
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  @Override
  public void robotInit() {
    gyro.reset();
    rightMEnc.setPosition(0.0);
    leftMEnc.setPosition(0.0);
    SmartDashboard.putNumber("gyro", gyro.getAngle());
  }

  @Override
  public void robotPeriodic() {
    updateOdometry();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // if (!isAligned()) {
    //   alignRobot(getHorizontalOffset());
    // } else {
    //   rightMotors.set(0);
    //   leftMotors.set(0);
    // }

    // var color = getDetectedColor();
    // var proximity = getProximity();
    // logToDashboard(proximity, color);

    turnToAngle(90);
    //moveDistance(1);
    logToDashboard(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), getHeading());
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
    return colorSense.getProximity();
  }

  /**
   * Determines which color is closest to the color detected by the sensor
   * 
   * @return The color that is closest to what the color sensor detects (RED,
   *         BLUE, OTHER or NONE)
   */
  public ColorChoices getDetectedColor() {
    // TODO write method
    RawColor c = colorSense.getRawColor();
    int red = c.red;
    int green = c.green;
    int blue = c.blue;
    SmartDashboard.putNumber("REDDDDDD", red);
    SmartDashboard.putNumber("GREEEEEEEEN", green);
    SmartDashboard.putNumber("BLUEeee", blue);

    if (red > blue) {
      return ColorChoices.RED;
    } else if (blue > red) {
      return ColorChoices.BLUE;
    } else {
      return ColorChoices.NONE;
    }  
  }

  /*
  Emojis go here
  >:)
  \(^V^)/
  */

  
  /**
   * Logs important color sensor values to SmartDashboard
   * 
   * @param proximity How close the nearest object is to the sensor
   * @param color     What color is detected by the sensor
   */
  public void logToDashboard(int proximity, ColorChoices color) {
    switch (color) {
      case BLUE:
        SmartDashboard.putString("Color", "Blue");
        break;
      case RED:
        SmartDashboard.putString("Color", "Red");
        break;
      case NONE:
        SmartDashboard.putString("Color", "None");
        break;
      default:
        SmartDashboard.putString("Color", "None");
        break;
    }
    SmartDashboard.putNumber("Proximity", proximity);
  }

  /* LIMELIGHT STATION */

  /**
   * Determines whether or not there is a target (as determined by limelight)
   * 
   * @return whether or not a target is visible in the frame
   */
  public boolean existsTarget() {
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
    double x = tx.getDouble(0.0);
    return x;
  }

  //e

  /**
   * Determines whether or not the robot is aligned with the goal
   * 
   * @return if the robot is aligned with the goal
   */
  public boolean isAligned() {
    double x = getHorizontalOffset();
    if (Math.abs(x) <= 5)
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
      rightMotors.set(-.05);
      leftMotors.set(-.05);
    } else if (horizontalError > 0) {
      rightMotors.set(0.05);
      leftMotors.set(0.05);
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
  public double getHeading() { // Heidi
    // TODO write method
    double heading = gyro.getAngle();
    return heading;
  }

  /**
   * Turns the robot to a specified angle
   * 
   * @param degrees angle in degrees
   */
  public void turnToAngle(double degrees) {
    // TODO write method
    //use getHeading() here
    double heading = getHeading();
    SmartDashboard.putNumber("heading", heading);
    if (heading != degrees && degrees > 0) {
      // Reminder: Motors are positioned in opposite directions, so yah
      leftMotors.set(0.05);
      rightMotors.set(0.05);
    } else if (heading != degrees && degrees < 0) {
      leftMotors.set(-0.05);
      rightMotors.set(-0.05);
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
  public void moveDistance(double distance) { //i think we're good
    // TODO write method <3 <3 <3 <3 <3
    distance = distance/2.1;
    if (getDistanceTraveled() < Math.abs(distance) && distance > 0) {
      leftMotors.set(-0.1);
      rightMotors.set(0.1);
    } else if (getDistanceTraveled() < Math.abs(distance) && distance < 0) {
      leftMotors.set(0.1);
      rightMotors.set(-0.1);
    } else {
      leftMotors.set(0);
      rightMotors.set(0);
    }
  }

  /**
   * Updates the odometry with distance measurements from encoders
   * 
   */
  public void updateOdometry() { // Sophia
    double gearRatio = Math.PI * 0.1524 / 10.81;
    odometry.update(gyro.getRotation2d(), leftMEnc.getPosition() * gearRatio, rightMEnc.getPosition() * gearRatio);
  }

  /**
   * Gets the distance of the robot from its starting point
   * 
   * @return distance in meters
   */
  public double getDistanceTraveled() { // Miriam
    // TODO write method
    //Use the odometry to get the distance x and y from our starting point 
    //Then, compute the hypotenuse to get the actual distance
    double x = odometry.getPoseMeters().getX();
    double y = odometry.getPoseMeters().getY();
    double hypotenuse = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    return hypotenuse;
  }

  /**
   * Logs important position values to SmartDashboard
   */
  public void logToDashboard(double xPos, double yPos, double rotation) {
    // TODO write method
    SmartDashboard.putNumber("XPOS", xPos);
    SmartDashboard.putNumber("YPOS", yPos);
    SmartDashboard.putNumber("Rotation", rotation);
  }
}
