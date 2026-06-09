package frc.robot.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Conversions {

    private static final double MIN_DISTANCE = 0.9;
    private static final double MAX_DISTANCE = 4.0;
    private static final double MIN_RPM = 1650.0;
    private static final double MAX_RPM = 2600.0;

    private static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP =
            new InterpolatingDoubleTreeMap();

    static {
        SHOOTER_RPM_MAP.put(0.9, 1650.0);
        SHOOTER_RPM_MAP.put(1.2, 1650.0);
        SHOOTER_RPM_MAP.put(1.4, 1720.0);
        SHOOTER_RPM_MAP.put(2.0, 1920.0);
        SHOOTER_RPM_MAP.put(2.3, 1960.0);
        SHOOTER_RPM_MAP.put(2.5, 2000.0);
        SHOOTER_RPM_MAP.put(2.8, 2120.0);
        SHOOTER_RPM_MAP.put(2.9, 2120.0);
        SHOOTER_RPM_MAP.put(3.0, 2130.0);
        SHOOTER_RPM_MAP.put(3.1, 2150.0);
        SHOOTER_RPM_MAP.put(3.2, 2150.0);
        SHOOTER_RPM_MAP.put(3.3, 2190.0);
        SHOOTER_RPM_MAP.put(3.4, 2210.0);
        SHOOTER_RPM_MAP.put(3.6, 2300.0);
        SHOOTER_RPM_MAP.put(4.0, 2600.0);
    }

    public static double distanceToRPM(double actualMeters) {
        if (actualMeters <= MIN_DISTANCE) return MIN_RPM;
        if (actualMeters >= MAX_DISTANCE) return MAX_RPM;
        return SHOOTER_RPM_MAP.get(actualMeters);
    }
}