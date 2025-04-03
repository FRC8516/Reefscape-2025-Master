package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Robot calibrations.
 */
public class Calibrations {

    /**
     * Field calibrations.
     */
    public static class FieldCalibrations {

        // The LL AprilTag ID is an integer
        // Encode the left branch as a 0 and the right branch as a 1 to match LL
        // Red alliance reef tags: 6,7,8,9,10,11
        // Blue alliance reef tags: 17,18,19,20,21,22
        
        public static final Map<Integer, Map<Integer, Double>> m_coralReefTargets;

        static {
            Map<Integer, Map<Integer, Double>> tempMap = new HashMap<>();
            tempMap.put(6, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(7, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(8, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(9, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(10, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(11, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(17, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(18, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(19, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(20, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(21, createImmutableMap(0, -27.78, 1, 0.91));
            tempMap.put(22, createImmutableMap(0, -27.78, 1, 0.91));

            m_coralReefTargets = Collections.unmodifiableMap(tempMap);
        }
        
        private static <K, V> Map<K, V> createImmutableMap(K k1, V v1, K k2, V v2) {
            Map<K, V> map = new HashMap<>();
            map.put(k1, v1);
            map.put(k2, v2);
            return Collections.unmodifiableMap(map);
        }

        public static final List<Integer> m_validTagIds = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    }

    /**
     * Driver calibrations.
     */
    public static class DriverCalibrations {

        /* Max speed, in meters per seocond, of the robot */
        public static final double kmaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /* 3/4 max angular velocity, in rotations per second, of the robot */
        public static final double kmaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* P-gain for rotational controller using LL tx as feedback */
        public static final double kAprilTagRotationAlignmentKP = 0.05;
        
        /* When the LL doesn't see a tag, use this value...which essentially sets the error to 0 */
        public static final double kLimelightDefaultKTx = 0;

        /* P-gain for robot-centric X-translational profiled PID controller */
        public static final double kAprilTagTranslationXAlignmentKP = 0.1;

        /* D-gain for robot-centric X-translational profiled PID controller */
        public static final double kAprilTagTranslationXAlignmentKD = 0.005;

        /* Profiled PID controller on-target threshold in degrees*/
        public static final double kAprilTagTranslationXOnTarget = 1.75;        
        
        /* Robot-centric X-translational controller - add a little Y-translation to stay flush to the coral reef */
        public static final double kAprilTagTranslationYRate = -0.1;

        /* Amount of rumble (0-1) to apply to the driver controller */
        public static final double kControllerRumbleValue = 1;

        /* Time to apply the controller rumble for before it turns off */
        public static final double kControllerRumblePulseTime = 0.1;
        
    }

}