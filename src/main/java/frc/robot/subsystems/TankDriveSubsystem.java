// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Util.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class TankDriveSubsystem extends SubsystemBase {
  private static final double whiteWheelDiameter = inchesToMeters(6.0); 
  public final MotorGroup leftWheels, rightWheels;
  public final PIDController pidController;
  public final AHRS navX;
  private final double initialHeading;
  public double leftSpeed = 0.0, rightSpeed = 0.0;
  private IdleMode idleMode;

  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem(MotorGroup leftWheels, MotorGroup rightWheels, PIDController pidController, AHRS navX) {
      this.leftWheels = leftWheels;
      this.rightWheels = rightWheels;
      this.idleMode = null;
      this.leftWheels.setInverted(true);
      this.pidController = pidController;
      this.navX = navX;
      this.navX.calibrate();
      this.initialHeading = this.getHeading();
  }

  public void setSpeeds(double left, double right) {
      this.leftSpeed = left;
      this.rightSpeed = right;
  }

  public void setIdleMode(IdleMode mode) {
      this.idleMode = mode;
  }

  @Override
  public void periodic() {
      if (this.idleMode != null) {
        this.leftWheels.setIdleMode(this.idleMode);
        this.rightWheels.setIdleMode(this.idleMode);
      }
      this.smartPID(this.leftWheels, this.leftSpeed);
      this.smartPID(this.rightWheels, this.rightSpeed);
      // this.leftWheels.set(leftSpeed);
      // this.rightWheels.set(rightSpeed);
  }

  private void smartPID(MotorGroup motorGroup, double setPoint) {
    if (!shouldBrake(boolNegate(motorGroup.get(), !motorGroup.leader.getInverted()), setPoint)) {
        if (this.idleMode == null) motorGroup.setIdleMode(IdleMode.kBrake);
        motorGroup.set(setPoint);
    } else {
        if (this.idleMode == null) motorGroup.setIdleMode(IdleMode.kCoast);
        motorGroup.set(this.pidController.calculate(motorGroup.get(), setPoint));
    }
  }

  public double getInitialHeading() {
    return this.initialHeading;
  }

  public double getHeading() {
    return (double) this.navX.getYaw();
  }

  private static boolean shouldBrake(double current, double desired) {
    return Math.abs(current) > Math.abs(desired) || Math.abs(current) > -Math.abs(desired);
  }

  // sign ? -x : x
  private static double boolNegate(double x, boolean sign) {
    return sign ? -x : x;
  }
}
