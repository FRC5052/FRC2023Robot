package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Util {
    public static double inchesToMeters(double inches){
        return inches / 39.37;
    }

    public static double metersToInches(double meters) {
        return meters * 39.37;
    }

    // sign ? -x : x
    public static double boolNegate(double x, boolean sign) {
        return sign ? -x : x;
    }

    public static long boolNegate(long x, boolean sign) {
        return sign ? -x : x;
    }

    public static double getSeconds(int ticks) {
        return ((double)ticks)*TimedRobot.kDefaultPeriod;
    }
}
