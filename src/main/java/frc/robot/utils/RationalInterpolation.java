package frc.robot.utils;

public final class RationalInterpolation {
	private static final double t1 = 1.42;
	private static final double t2 = 2.56;
	private static final double t3 = 4.50;
	private static final double r1 = 45;
	private static final double r2 = 27;
	private static final double r3 = 19.5;

	private static double inter12(double t) {
		return r2 + (r2-r1)/(((t-t1)/(t-t2))*(1-((r2-r1)/r2)) - 1) ;
	}
	private static double inter23(double t) {
		 return r3 + (r3-r2)/(((t-t2)/(t-t3))*(1-((r3-r2)/r3)) - 1);
	}

	/**
	 *
	 * @param distance
	 * @return shooter angle
	 */
	public static double getShooterAngle(double distance) {
		return inter23(distance) + (inter23(distance)-inter12(distance))/(((distance-t1)/(distance-t3))*(1-(inter23(distance)-inter12(distance))/(inter23(distance)-r2))-1);
	}
}
