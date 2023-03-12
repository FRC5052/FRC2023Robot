package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.RobotContainer.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class AutonCommand extends TimedCommand {
    private boolean onRamp;

    private RelativeEncoder rEncoder;
    private RelativeEncoder lEncoder;
    private double numberOfMotorRevolutionsNeeded = 80.0;
    private double feetPerMotorRevolution = 0.16095;
    private double wheelTurnPerMotorTurn = 0.10247;
    
    public AutonCommand() {
        rEncoder = robot.tankDriveSubsystem.rightWheels.leader.getEncoder();
        lEncoder = robot.tankDriveSubsystem.leftWheels.leader.getEncoder();
    }

    @Override
    public void initialize() {
        super.initialize();
        this.onRamp = false;

        rEncoder.setPosition(0.0);
        lEncoder.setPosition(0.0);
        robot.tankDriveSubsystem.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void execute() {
        if (robot.turretController.getAButton() || robot.driveController.getAButton()) this.cancel();

        // if (this.getSeconds() < 0.5) { // let go of cube
        //     robot.turretClawSubsystem.openClaw();
        //     robot.turretArmSubsystem.lock();
        // } else if (this.getSeconds() < 7.5) {
        //     // robot.turretPivotSubsystem.turn(0.3);
        //     robot.turretClawSubsystem.stopClaw();
        //     // delay
// }


        
        if (!this.onRamp) {
            if(this.getSeconds() < 1.5){
                robot.turretClawSubsystem.closeClaw();
                robot.turretClawSubsystem.setIntake();
            }
            else {
                robot.turretClawSubsystem.openClaw();
                robot.turretClawSubsystem.setHome();
                if (Math.abs(robot.tankDriveSubsystem.navX.getPitch()) > 2.0) {
                    this.onRamp = true;
                } else {
                    double motorRevolutionAverage = (Math.abs(rEncoder.getPosition()) + Math.abs(lEncoder.getPosition())) / 2;
                    System.out.printf("R: %f, L: %f\n", rEncoder.getPosition(), lEncoder.getPosition());
                    if(motorRevolutionAverage < numberOfMotorRevolutionsNeeded){
                        robot.tankDriveSubsystem.setSpeeds(-0.85, -0.85);
                    }
                    else {
                        robot.tankDriveSubsystem.setSpeeds(0.0, 0.0);
                        robot.tankDriveSubsystem.setIdleMode(IdleMode.kBrake);
                    }
                }
            }

        } else {
            if (this.getSeconds() < 15.0){    
                robot.tankDriveSubsystem.setIdleMode(IdleMode.kBrake);
                double value = MathUtil.applyDeadband(-robot.tankDriveSubsystem.navX.getPitch()/110.0, 0.05);
                robot.tankDriveSubsystem.setSpeeds(value, value);
            } else {
                robot.tankDriveSubsystem.setSpeeds(0.0, 0.0);
                robot.tankDriveSubsystem.setIdleMode(IdleMode.kBrake);
            }
        }
        super.execute();
    }
}
