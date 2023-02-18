// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.motor.MotorGroup;

public class TankDriveSubsystem extends SubsystemBase {
  public final AHRS navX;
  public final MotorGroup leftWheels, rightWheels;
  public final PIDController pidController;
  public double leftSpeed = 0.0, rightSpeed = 0.0;

  // whoever instantiated the Gyro i fixed it- just fix the port bec idk where its plugged in
  // to use Gyro go here https://first.wpi.edu/Images/CMS/First/WPI_Robotics_Library_Users_Guide.pdf | pg 19

  // try and get it to work with static, or find a way to get the rotation in 2D for the odometry 
  // this does the odometry using the gyro above- if you change the gyro try and fix this pls
  private final DifferentialDriveOdometry m_odometry;
  private CANSparkMax leftLeader;
  private CANSparkMax rightLeader;
  private RelativeEncoder lEncoder;
  private RelativeEncoder rEncoder;

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem(MotorGroup leftWheels, MotorGroup rightWheels, PIDController pidController, AHRS navX
    ) {
      this.leftWheels = leftWheels;
      this.rightWheels = rightWheels;
      this.leftWheels.setIdleMode(IdleMode.kBrake);
      this.rightWheels.setIdleMode(IdleMode.kBrake);
      this.leftWheels.setInverted(true);
      this.pidController = pidController;
      this.leftLeader = leftLeader;
      this.rightLeader = rightLeader;
      lEncoder = leftLeader.getEncoder();
      rEncoder = rightLeader.getEncoder();

      rEncoder.setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
      lEncoder.setPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
      rEncoder.setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor/60);
      lEncoder.setVelocityConversionFactor(Constants.kLinearDistanceConversionFactor/60);

      this.navX = navX;
      navX.reset();
      this.navX.calibrate();
      resetEncoders();

      m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), this.lEncoder.getPosition(), this.rEncoder.getPosition());

      // commented this our caus above is a Static Gyro thats more cooler and swag

  }

  public void setBreakMode(MotorGroup leftWheels, MotorGroup rightWheels){
    leftWheels.setIdleMode(IdleMode.kBrake);
    rightWheels.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(MotorGroup leftWheels, MotorGroup rightWheels){
    leftWheels.setIdleMode(IdleMode.kCoast);
    rightWheels.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    rEncoder.setPosition(0);
    lEncoder.setPosition(0);
  }

  public double getRightEncoderPosition() {
    return rEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return lEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return lEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return lEncoder.getVelocity();
  }

  public double getTurnRate(){
    return -navX.getRate();
  }

  public void setSpeeds(double left, double right) {
      this.leftSpeed = left;
      this.rightSpeed = right;
  }

  public double getHeading() {
    return this.navX.getRotation2d().getDegrees();
  }
  // might have to make this method static

  @Override
  public void periodic() {
      this.smartPID(leftWheels, leftSpeed);
      this.smartPID(rightWheels, rightSpeed);
      // this.leftWheels.set(leftSpeed);
      // this.rightWheels.set(rightSpeed);

      // for odometry

      m_odometry.update(navX.getRotation2d(), lEncoder.getPosition(), rEncoder.getPosition());
      SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
      SmartDashboard.putNumber("Right encoder value meters", getRightEncoderPosition());
      SmartDashboard.putNumber("Gyro heading", getHeading());
      
  }

  private void smartPID(MotorGroup motorGroup, double setPoint) {
    if (shouldBrake(motorGroup.get(), setPoint)) {
        motorGroup.setIdleMode(IdleMode.kBrake);
        motorGroup.set(setPoint);
    } else {
        motorGroup.setIdleMode(IdleMode.kCoast);
        motorGroup.set(this.pidController.calculate(motorGroup.get(), setPoint));
    }
  }

  private boolean shouldBrake(double current, double desired) {
    return Math.abs(current) > Math.abs(desired) || Math.abs(current) > -Math.abs(desired);
  }
}
