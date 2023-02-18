// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // false for Rocket League controls, true for Arcade controls
    public static final boolean useAlternateControls = false;

    public static final int leftLeaderID = 2;
    public static final int leftFollowerID = 1;
    public static final int rightLeaderID = 3;
    public static final int rightFollowerID = 4;

    public static final int armMotorID = 5;
    public static final int[] armEncoderPorts = {0, 1, 2, 3};
    public static final int armLimSwitchPort = 4;

    public static final int pivotMotorId = 6;


// stuff below is constants for Auton dont touch unless u know what ur doing 
// the "k" just means that it's a constant- something that doesnt change (ex. the length of 1ft will never change)

    // once the robot is put together we can calculate the info using SysID before and plug it in 

    public static final double ksVolts = 0;
    public static final double KvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;

    // this is just the horizontal distance between the front wheel and back wheel IN METERS (ex. the different from Front Left & Back Left)
    // 11in --> meters
    public static final double kTrackWidthMeters = Units.inchesToMeters(17.75);

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    // setting max velocity & acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;


    // baseline values for RAMSETE Folloer in meters/second
    public static final double kRamseteB = 2;
    public static final double kRamsateZeta = 0.7;

    public static final double kGearRatio = 5.12;
    public static final double kWheelRadiusInches = 6;

    //distance converter from ticks to meters
    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1/(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));


    // to convert ticks to degrees :
    /*
     *  degrees = ( ticks / TPR ) * 360
     * 
     *  to get a 90 degree turn, get 1/4th of whatever your total TPR is 
     * 
     */
}
