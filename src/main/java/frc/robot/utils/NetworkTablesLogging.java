package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public interface NetworkTablesLogging {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    default void log(String table, String key, double val) {
        inst.getTable(table).getEntry(key).setDouble(val);
    }
    default void log(String table, String key, String val) {
        inst.getTable(table).getEntry(key).setString(val);
    }
    default void log(String table, String key, boolean val) {
        inst.getTable(table).getEntry(key).setBoolean(val);
    }
    default void log(String table, String key, int val) {
        inst.getTable(table).getEntry(key).setNumber(val);
    }
    default void log(String table, String key, Pose2d val) {
        inst.getTable(table).getEntry(key).setDoubleArray(new double[] {val.getTranslation().getX(), val.getTranslation().getY(), val.getRotation().getRadians()});
    }
    default void log(String table, String key, Pose3d val) {
    double[] pose3dArray = new double[] {
        val.getTranslation().getX(),
        val.getTranslation().getY(),
        val.getTranslation().getZ(),
    };
    inst.getTable(table).getEntry(key).setDoubleArray(pose3dArray);
}
}