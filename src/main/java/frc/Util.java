package frc;

public class Util {
    public static double inchesToMeters(double inches){
        return inches / 39.37;
    }

    public static double metersToInches(double meters) {
        return meters * 39.37;
    }

    // sign ? -x : x
    private static double boolNegate(double x, boolean sign) {
        return sign ? -x : x;
    }

    private static long boolNegate(long x, boolean sign) {
        return sign ? -x : x;
    }
}
