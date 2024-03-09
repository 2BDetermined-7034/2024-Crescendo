package frc.robot.utils;

import java.util.Arrays;


public class ThieleInterpolation {
	/**
	 * Number of Data points
	 */
	private static final int N = 5;
	/**
	 * Size of Cache for rho
	 */
	private static final int N2 = (N * (N - 1) / 2);
	/**
	 * Rational Interpolation input dataset
	 */
	private static final double[] xvals = new double[N];
	/**
	 * Rational Interpolation output dataset
	 */
	private static final double[] yvals = new double[N];
	/**
	 * Cache list for rho
	 */
	private static final double[] rvals = new double[N2];
	static {
		initializeDataSet();
	}

	private static void initializeDataSet() {
		//It's convenient for the cache to be NaN
		Arrays.fill(rvals, Double.NaN);

		xvals[0] = 1.42;
		xvals[1] = 2.56;
		xvals[2] = 3.4056;
		xvals[3] = 4.50;
		xvals[4] = 5.50;

		yvals[0] = 45;
		yvals[1] = 27;
		yvals[2] = 21;
		yvals[3] = 19.5;
		yvals[4] = 17.5;
	}

	/**
	 * Recursive Reciprocal Difference Function
	 * Uses base memoization to cache values
	 *
	 * @param x list of input values x_i
	 * @param y list of function output values f(x_i)
	 * @param r cache
	 * @param i step
	 * @param n order of reciprocal difference
	 * @return reciprocal difference of inputs
	 */
	private static double rho(double[] x, double[] y, double[] r, int i, int n) {
		if (n < 0) {
			return 0;
		}

		if (n == 0) {
			return y[i];
		}

		int idx = (N - 1 - n) * (N - n) / 2 + i;
		//Double NaN comparison returns false
		if (r[idx] != r[idx]) {
			r[idx] = (x[i] - x[i + n])
					/ (rho(x, y, r, i, n - 1) - rho(x, y, r, i + 1, n - 1))
					+ rho(x, y, r, i + 1, n - 2);
		}

		return r[idx];
	}

	/**
	 * Recursively calculates outputs of a Thiele Rational Interpolation function
	 * @param x list of input values x_i
	 * @param y list of function output values f(x_i)
	 * @param r rho cache
	 * @param xin input value of x
	 * @param n Recursion step
	 * @return interpolated value at xin
	 */
	static double thiele(double[] x, double[] y, double[] r, double xin, int n) {
		if (n > N - 1) {
			return 1;
		}
		return rho(x, y, r, 0, n) - rho(x, y, r, 0, n - 2)
				+ (xin - x[n]) / thiele(x, y, r, xin, n + 1);
	}

	/**
	 * Gets the ideal shooter angle for a given distance away from the speaker using rational interpolation
	 * @param distance in meters from speaker as reported by PhotonVision
	 * @return optimal shooter angle in degrees
	 */
	public static double shooterDistanceToAngle(double distance) {
		return thiele(xvals, yvals, rvals, distance, 0);
	}

    /*

    public static void main(String[] args) {
        for(int i = 0;i < 300;i++) {
            System.out.printf("%16.14f    %.2f %n", thiele(xvals, yvals, rvals, i * 0.02, 0), i * 0.02);
        }
    }

   */
}