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

    public void turn(double amount) {
        this.turretMotor.set(amount*0.25);
        // desiredHeadingOffset += 0.5;
    }

    @Override
    public void periodic() {

        // turretDegrees = (turretEncoder.getPosition()/140) * 360;
        // robotDifferenceHeading = (robot.tankDriveSubsystem.getInitialHeading() - robot.tankDriveSubsystem.getHeading()) + desiredHeadingOffset;
        
        // System.out.printf("Robot Difference Heading = %f\n", robotDifferenceHeading);
        // System.out.printf("turretDegrees = %f\n", turretDegrees);

        // if (!(turretDegrees < robotDifferenceHeading+2)){
        //     this.turretMotor.set(-0.3);
        // }else if (!(turretDegrees > robotDifferenceHeading-2)){
        //     this.turretMotor.set(0.3);
        // }// if ((turretDegrees < robotDifferenceHeading+5) && (turretDegrees > robotDifferenceHeading-5)){
        // else {
        //     this.turretMotor.set(0);
        // }
    }

    public void stopTurning() {
        this.turretMotor.stopMotor();
    }



}
