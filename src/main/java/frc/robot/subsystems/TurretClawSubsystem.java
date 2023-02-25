// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class TurretClawSubsystem extends SubsystemBase {
  private final MotorGroup motors;


  public TurretClawSubsystem(MotorGroup motors) {
    this.motors = motors;
  }

  public void openClaw() {
    this.motors.set(0.1);
  }

  public void closeClaw() {
    this.motors.set(-0.1);
  }

  public void stopClaw() {
    this.motors.set(0.0);
  }

  @Override
  public void periodic() {
    
  }
}
