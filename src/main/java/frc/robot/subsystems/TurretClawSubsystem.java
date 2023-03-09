// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class TurretClawSubsystem extends SubsystemBase {
  private static double maxEncoderValue = 1.0;
  private final MotorGroup motors;
  private final AbsoluteEncoder encoder;
  private boolean shouldReset = false;

  public TurretClawSubsystem(MotorGroup motors) {
    this.motors = motors;
    this.motors.setIdleMode(IdleMode.kBrake);
    this.encoder = this.motors.leader.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public void openClaw() {
    this.motors.set(-0.15);
  }

  public void closeClaw() {
    this.motors.set(0.15);
  }

  public void stopClaw() {
    this.motors.set(0.0);
  }

  public void resetClaw() {
    this.shouldReset = true;
  }

  @Override
  public void periodic() {
    if (this.shouldReset) {
      if (this.encoder.getPosition() < maxEncoderValue) {
        this.motors.set(0.1);
      } else {
        this.shouldReset = false;
      }
    }
  }
}
