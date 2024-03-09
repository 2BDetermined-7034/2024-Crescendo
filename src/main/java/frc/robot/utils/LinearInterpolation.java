package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class LinearInterpolation {
	public Translation2d[] vertices;

	public LinearInterpolation() {}

	public double get(double x) {
		Translation2d a = vertices[0], b = vertices[1];
		for (int i = 0; i < vertices.length - 1; ++i) {
			a = vertices[i];
			b = vertices[i + 1];
			if (x < b.getX()) break;
		}

		return a.getY() + (b.getY() - a.getY()) / (b.getX() - a.getX()) * (x - a.getX());
	}
}
