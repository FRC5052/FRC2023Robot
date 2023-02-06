// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class TankDriveSubsystem extends SubsystemBase {
  public final MotorGroup leftWheels, rightWheels;
  public final PIDController pidController;
  public final AHRS navX;
  public double leftSpeed = 0.0, rightSpeed = 0.0;
  /** Creates a new TankDriveSubsystem. */
  public TankDriveSubsystem(MotorGroup leftWheels, MotorGroup rightWheels, PIDController pidController, AHRS navX) {
      this.leftWheels = leftWheels;
      this.rightWheels = rightWheels;
      this.leftWheels.setIdleMode(IdleMode.kBrake);
      this.rightWheels.setIdleMode(IdleMode.kBrake);
      this.rightWheels.setInverted(true);
      this.pidController = pidController;
      this.navX = navX;
      this.navX.calibrate();
  }

  @Override
  public void periodic() {
      this.leftWheels.set(this.pidController.calculate(this.leftWheels.get(), leftSpeed));
      this.rightWheels.set(this.pidController.calculate(this.rightWheels.get(), rightSpeed));
      // this.leftWheels.set(leftSpeed);
      // this.rightWheels.set(rightSpeed);
  }
}
