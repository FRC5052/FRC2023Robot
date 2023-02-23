package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.robot;
import com.revrobotics.RelativeEncoder;

public class TurretPivotSubsystem extends SubsystemBase {
    public final CANSparkMax turretMotor;
    private RelativeEncoder turretEncoder;

    private double turretDegrees;

    private double robotDifferenceHeading;

    private double desiredHeading;
    private double desiredHeadingOffset;

    //double x = robot.controller.getPOV();


    //double rotX = Math.cos(-robotAngle) - Math.sin(-robotAngle);
    //double rotY = x * Math.sin(-robotAngle) + y * Math.cos(-robotAngle);

    public TurretPivotSubsystem (CANSparkMax turretMotor) {
        this.turretMotor = turretMotor;
        this.turretEncoder = turretMotor.getEncoder();
        this.turretEncoder.setPosition(0);
    }

    public void turnRight() {
        // turretMotor.set((robot.turretController.getRightTriggerAxis())*0.5);
        desiredHeadingOffset++;
    }

    public void turnLeft() {
        //turretMotor.set((-robot.turretController.getLeftTriggerAxis())*0.5);
        desiredHeadingOffset--;
    }

    public void turn90(){
        desiredHeadingOffset = 90;
    }

    public void turn180(){
        desiredHeadingOffset = 180;
    }
    
    public void turn270(){
        desiredHeadingOffset = 270;
    }

    @Override
    public void periodic() {

        // turretDegrees = (turretEncoder.getPosition()/140) * 360;
        // robotDifferenceHeading = (robot.tankDriveSubsystem.getInitialHeading() - robot.tankDriveSubsystem.getHeading()) + desiredHeadingOffset;
        
        // System.out.printf("Robot Difference Heading = %f\n", robotDifferenceHeading);
        // System.out.printf("turretDegrees = %f\n", turretDegrees);

        // if (!(turretDegrees < robotDifferenceHeading+0.1)){
        //     this.turretMotor.set(-0.2);
        // }else if (!(turretDegrees > robotDifferenceHeading-0.1)){
        //     this.turretMotor.set(0.2);
        // }// if ((turretDegrees < robotDifferenceHeading+5) && (turretDegrees > robotDifferenceHeading-5)){
        // else {
        //     this.turretMotor.set(0);
        // }
    }

    public void stopTurning() {
        this.turretMotor.stopMotor();
    }



}
