package frc.lib.math;

public class conversions {

    private final double CountsPerRev = 2048;


    /**
     * Converts from angle to counts
     * @param angle
     * @param gearRatio
     * @return Falcon motor counts
     */
    public double angleToCounts(double angle, double GearRatio) {
        return (angle / 360.0 * CountsPerRev) / GearRatio;
    }

    /**
     * Converts from counts to angle
     * @param counts
     * @param gearRatio
     * @return Angle of Falcon Motor
     */
    public double countsToAngle(double counts, double GearRatio) {
        /* test */
        return counts * GearRatio / CountsPerRev * 360.0; // flipped from angletocounts
    }

}
