// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.MotorGroup;

public class FlipperClawSubsystem extends SubsystemBase {
  private static double maxEncoderValue = 1.0;
  private final MotorGroup clawMotors;
  private final RelativeEncoder clawMotorEncoder;
  private final CANSparkMax clawFlipMotor;
  private final RelativeEncoder clawFlipMotorEncoder;
  private boolean shouldReset = false;

  private double clawFlipDesiredPosition = 1;
  private static double clawFlipintakePosition = 23.5;
  private static double clawFlipHomePosition = 10;
  private boolean clawFlipAtDesiredPosition = false;

  private double clawDesiredPosition = 0;
  private static final double clawOpenPosition = -.95;
  private static final double clawClosedPosition = 0.8;
  private boolean clawAtDesiredPosition = false;

  public FlipperClawSubsystem(MotorGroup clawMotors, CANSparkMax clawFlipMotor) {
    this.clawMotors = clawMotors;
    this.clawMotors.setIdleMode(IdleMode.kBrake);
    this.clawMotorEncoder = clawMotors.leader.getEncoder();

    this.clawFlipMotor = clawFlipMotor;
    this.clawFlipMotorEncoder = clawFlipMotor.getEncoder();
  }

  public void incrementOpen() {
    this.clawMotors.leader.set(-0.06);
  }

  public void incrementClose() {
    this.clawMotors.leader.set(0.06);
  }

  public void incrementOpenBigga() {
    this.clawMotors.leader.set(-0.1);
  }

  public void incrementCloseBigga() {
    this.clawMotors.leader.set(0.1);
  }

  public void incrementClawUp(){
    this.clawFlipMotor.set(-0.15);
  }

  public void incrementClawDown(){
    this.clawFlipMotor.set(.15);
  }

  public void setIntake(){
    clawFlipDesiredPosition = clawFlipintakePosition;
  }

  public void setHome(){
    clawFlipDesiredPosition = clawFlipHomePosition;
  }

  public void setDumping(){
    clawFlipDesiredPosition = 0;
  }

  public void stopFlipClaw(){
    this.clawFlipMotor.set(0);
  }

  public void openClaw(){
    clawDesiredPosition = clawOpenPosition;
  }

  public void closeClaw(){
    clawDesiredPosition = clawClosedPosition;
  }

  public void stopClaw() {
    this.clawMotors.leader.set(0.0);
  }

  public void resetClaw() {
    this.shouldReset = true;
  }

  public double getClawFlipPosition(){
    return clawFlipMotorEncoder.getPosition();
  }

  public double getClawPosition(){
    return clawMotorEncoder.getPosition();
  }

  public boolean clawIsAtDesiredPos() {
    return this.clawAtDesiredPosition;
  }

  public boolean clawFlipAtDesiredPos() {
    return this.clawFlipAtDesiredPosition;
  }

  public double PThisJawn(double cur, double des){
    return((des-cur) * 0.15);
  }

  @Override
  public void periodic() {
    if (this.shouldReset) {
        this.clawMotors.set(0.1);
    } else {
        this.shouldReset = false;
    }
    this.clawFlipAtDesiredPosition = false;
    if(this.clawFlipDesiredPosition < this.getClawFlipPosition()-2) {
      this.incrementClawUp();
    } else if(this.clawFlipDesiredPosition > this.getClawFlipPosition()+2) {
      this.incrementClawDown();
    } else {
      this.clawFlipAtDesiredPosition = true;
      this.stopFlipClaw();
    }

    
    this.clawAtDesiredPosition = false;

    if(this.clawDesiredPosition < this.getClawPosition()-.5) {
      this.incrementOpenBigga();
    } else if(this.clawDesiredPosition > this.getClawPosition()+.5) {
      this.incrementCloseBigga();
    } else if(this.clawDesiredPosition < this.getClawPosition()-.25) {
      this.incrementOpen();
    } else if(this.clawDesiredPosition > this.getClawPosition()+.25) {
      this.incrementClose();
    } else {
      this.clawAtDesiredPosition = true;
      this.stopClaw();
    }
    
    //this.clawMotors.leader.set(this.PThisJawn(this.getClawPosition(), this.clawDesiredPosition));
  }
}
