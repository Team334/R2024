package frc.robot.utils;

/**
 * Singleton class that helps retrieve limelight info from network tables.
 */
public class LimelightHelper {
    private static LimelightHelper _instance;

    public final static LimelightHelper getInstance() {
        if (_instance == null) {
            _instance = new LimelightHelper();
        }

        return _instance;
    }

    private LimelightHelper() {}
}
