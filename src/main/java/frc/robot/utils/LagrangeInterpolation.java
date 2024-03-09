package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Function;

public class LagrangeInterpolation {
	public Translation2d[] vertices;
	public Function<Double, Double> function = (Double x) -> {
		return x;
	};
	public Function<Double, Double> inverse = (Double x) -> {
		return x;
	};
	public LagrangeInterpolation() {}

	public double get(double x) {
		double sum = 0.0f;

		for (int i = 0; i < vertices.length; ++i) {
			double current = inverse.apply(vertices[i].getY());
			for (int j = 0; j < i; ++j) {
				current *= (x - vertices[j].getX()) / (vertices[i].getX() - vertices[j].getX());
			}

			for (int j = i + 1; j < vertices.length; ++j) {
				current *= (x - vertices[j].getX()) / (vertices[i].getX() - vertices[j].getX());
			}

			sum += current;
		}

		return function.apply(sum);
	}
}
