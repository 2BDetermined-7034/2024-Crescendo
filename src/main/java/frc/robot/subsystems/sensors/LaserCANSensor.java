package frc.robot.subsystems.sensors;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCANSensor extends SubsystemBase {
	private LaserCan lc;
	private int canID;
	private int latestDistance = 0;

	public LaserCANSensor(int canID) {
		lc = new LaserCan(canID);
		this.canID = canID;
	}

	@Override
	public void periodic() {
		LaserCan.Measurement measurement = lc.getMeasurement();
		if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
			//System.out.println("The target is " + measurement.distance_mm + "mm away!");
			SmartDashboard.putNumber(String.format("Lasercan %d Distance mm ", canID), measurement.distance_mm);
			latestDistance = measurement.distance_mm;
		} else {
			//System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
			// You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
		}
	}

	/**
	 * Returns the latest accurate measurement in mm
	 * @return distance (mm)
	 */
	public int getLatestMeasurement() {
		return latestDistance;
	}


}

