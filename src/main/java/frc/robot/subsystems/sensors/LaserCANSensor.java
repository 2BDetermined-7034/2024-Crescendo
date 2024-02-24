package frc.robot.subsystems.sensors;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
public class LaserCANSensor {
	private double pastWeight, currentWeight;
	private LaserCan laser;
	private double previousDistance;

	/**
	 *
	 * @param pastDistanceWeight
	 * @param canID
	 * @param dimension
	 *
	 */

	public LaserCANSensor(double pastDistanceWeight, int canID, int dimension) {
		setPastDistanceWeight(pastDistanceWeight);

		laser = new LaserCan(canID);
		try {
			laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
			laser.setRangingMode(LaserCan.RangingMode.SHORT);
			laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, dimension, dimension));
		} catch (Exception e) {
			DriverStation.reportError("Config failed", true);
		}
	}

	public void setRegionOfInterest(int x, int y, int dimension) {
		try {
			laser.setRegionOfInterest(new LaserCan.RegionOfInterest(x, y, dimension, dimension));
		} catch (Exception e) {
			DriverStation.reportError("LaserCAN Error: ROI configuration failed", true);
		}
	}

	public void setTimingBudget(LaserCan.TimingBudget timingBudget) {
		try {
			laser.setTimingBudget(timingBudget);
		} catch (Exception e) {
			DriverStation.reportError("LaserCAN Error: TimingBudget configuration failed",true);
		}
	}

	public void setRangingMode(LaserCan.RangingMode rangingMode) {
		try {
			laser.setRangingMode(rangingMode);
		} catch (Exception e) {
			DriverStation.reportError("LaserCAN Error: Ranging mode configuration failed", true);
		}
	}

	public void setPastDistanceWeight(double pastDistanceWeight) {
		this.pastWeight = Math.min(Math.max(pastDistanceWeight, 0.0), 1.0);
		this.currentWeight = 1.0 - pastWeight;
	}

	public LaserCan.Measurement getMeasurement() {
		LaserCan.Measurement measurement = laser.getMeasurement();
		switch (measurement.status) {
			case LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT:
				break;
			case LaserCan.LASERCAN_STATUS_NOISE_ISSUE:
				DriverStation.reportError("LaserCAN Error: Noise Issue (Consider increasing timing budget)", true);
				break;
			case LaserCan.LASERCAN_STATUS_WEAK_SIGNAL:
				DriverStation.reportError("LaserCAN Error: Could not find target", true);
				break;
			case LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS:
				DriverStation.reportError("LaserCAN Error: Target is out of range (Could be overly reflective)", true);
				break;
			case LaserCan.LASERCAN_STATUS_WRAPAROUND:
				DriverStation.reportError("LaserCAN Error: Value read may have wrapped around", true);
				break;
			default:
				DriverStation.reportError("LaserCAN Error: Unknown Error", true);
				break;
		}
		return measurement;
	}
	public double getWeightedDistance() {
		LaserCan.Measurement measurement = getMeasurement();
		if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
			return previousDistance = previousDistance * pastWeight + (double)measurement.distance_mm * currentWeight;
		}

		return -1.0;
	}

}

