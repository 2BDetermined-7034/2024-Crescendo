package frc.robot.subsystems.sensors;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
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
			System.err.println("Config failed");
		}
	}

	public void setRegionOfInterest(int x, int y, int dimension) {
		try {
			laser.setRegionOfInterest(new LaserCan.RegionOfInterest(x, y, dimension, dimension));
		} catch (Exception e) {
			System.err.println("LaserCAN Error: ROI configuration failed");
		}
	}

	public void setTimingBudget(LaserCan.TimingBudget timingBudget) {
		try {
			laser.setTimingBudget(timingBudget);
		} catch (Exception e) {
			System.err.println("LaserCAN Error: TimingBudget configuration failed");
		}
	}

	public void setRangingMode(LaserCan.RangingMode rangingMode) {
		try {
			laser.setRangingMode(rangingMode);
		} catch (Exception e) {
			System.err.println("LaserCAN Error: Ranging mode configuration failed");
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
				System.err.println("LaserCAN Error: Noise Issue (Consider increasing timing budget)");
				break;
			case LaserCan.LASERCAN_STATUS_WEAK_SIGNAL:
				System.err.println("LaserCAN Error: Could not find target");
				break;
			case LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS:
				System.err.println("LaserCAN Error: Target is out of range (Could be overly reflective)");
				break;
			case LaserCan.LASERCAN_STATUS_WRAPAROUND:
				System.err.println("LaserCAN Error: Value read may have wrapped around");
				break;
			default:
				System.err.println("LaserCAN Error: Unknown Error");
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

