package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/*
 * This is for all the software noobs (minus dustin)
 * code this claw
 * imports + methods have been made along with motor instantiation
 * 
 * Things to Include:
 * make sure the claw can change between the Cone Claw & Cube Claw
 * make sure you can actually open & close both the claws perfectly -
 * make a way for both the claws to stay closed on default (or how the drivers wish)
 *  
 * 
 * you got this- Arav
 */


public class ClawSubsystem {

    /* 
    the methods below are jus templates, if you feel like
    a method isn't needed or should be changed, go for it
    */

    public final CANSparkMax baseClaw;
    public final CANSparkMax coneClaw;
    public final CANSparkMax cubeClaw;
 
    public final RelativeEncoder baseClawEncoder;
    public final RelativeEncoder coneClawEncoder;
    public final RelativeEncoder cubeClawEncoder;


    public ClawSubsystem(CANSparkMax clawMotorBase, CANSparkMax clawMotorCone, CANSparkMax clawMotorCube){
      /* the following motors are Neo 550 Brushless motors- 
         they are pretty much just mini CANSparkMaxs (the motors used for the wheels)
      */

        baseClaw = clawMotorBase;
        coneClaw = clawMotorCone;
        cubeClaw = clawMotorCube;
        
        /*  the following is the in-built hall encoder rings that are on all the 550's
         *  you can use the encoders to go to exact postions, measured in 'ticks' 
         *  this is more accurate than saying "set power for 2s", because the motor
         *  will keep running until the encoder has spun enough 'ticks'
         */

         // these are "relative" encoders because...they are relative
        
         baseClawEncoder = baseClaw.getEncoder();
         coneClawEncoder = coneClaw.getEncoder();
         cubeClawEncoder = cubeClaw.getEncoder();
    }

    public void changeClawType(){

    }

    public void closeClawCone(){

    }

    public void closeClawCube(){

    }

    public void stopClawCubeCone(){

    }

    public void stopBaseClaw(){

    }
}
