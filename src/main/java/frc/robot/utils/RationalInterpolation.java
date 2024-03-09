package frc.robot.utils;

public class RationalInterpolation {
	private double t1 = 0.92;
	private double t2 = 2.685;
	private double t3 = 4.135;
	private double r1 = 63;
	private double r2 = 25;
	private double r3 = 19;

	public RationalInterpolation(){}
	private double inter12(double t) {
		return r2 + (r2 - r1) / (t - t1) / (t - t3) * (1 - (r2 - r1)/ r2) - 1;
	}
	private double inter23(double t) {
		return r3 + (r3 - r2) / (t - t2) / (t - t3) * (1- (r3 - r2)/r3) - 1;
	}

	/**
	 *
	 * @param distance
	 * @return shooter angle
	 */
	public double getShooterAngle(double distance) {
		return inter23(distance) + (inter23(distance) - inter12(distance)) / (distance - t1)/(distance - t3) * (1 - inter23(distance) - inter12(distance) / inter23(distance) - r2) - 1;
	}
}
