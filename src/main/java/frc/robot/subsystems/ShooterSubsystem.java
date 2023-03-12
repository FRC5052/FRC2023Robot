// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax leftMotor, rightMotor, tosserMotor;
  private RelativeEncoder tosserEncoder;

  private double tosserDesiredPosition = 0;
  private boolean tosserAtDesiredPosition;
  private static final double tosserFeedingPosition = 2.3;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(CANSparkMax left, CANSparkMax right, CANSparkMax tosser) {
    this.leftMotor = left;
    this.rightMotor = right;

    this.tosserMotor = tosser;
    this.tosserEncoder = tosser.getEncoder();
    this.leftMotor.setIdleMode(IdleMode.kCoast);
    this.rightMotor.setIdleMode(IdleMode.kCoast);

    this.tosserAtDesiredPosition = false;
  }

  public void setSpeed(double speed) {
    this.leftMotor.set(-speed);
    this.rightMotor.set(speed);
  }

  public void tosserIncrementUp(){
    this.tosserMotor.set(0.05);
  }

  public void tosserIncrementDown(){
    this.tosserMotor.set(-0.05);
  }

  public void feed(){
    this.tosserDesiredPosition = tosserFeedingPosition;
  }
  
  public void down(){
    this.tosserDesiredPosition = 0;
  }

  public void stopTosser(){
    this.tosserMotor.set(0);
  }

  public double getTosserPosition(){
    return this.tosserEncoder.getPosition();
  }

  public boolean isAtDesiredPosition() {
    return this.tosserAtDesiredPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.tosserAtDesiredPosition = false;
    if(this.tosserDesiredPosition < this.getTosserPosition()-.2) {
      this.tosserIncrementDown();
    } else if(this.tosserDesiredPosition > this.getTosserPosition()+.2) {
      this.tosserIncrementUp();
    } else {
      this.tosserAtDesiredPosition = true;
      this.stopTosser();
    }
  }
}
