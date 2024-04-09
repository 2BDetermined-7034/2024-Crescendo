package frc.robot.utils.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Optional;

public class Pathfinder {
	public Face[] faces;
	public Translation2d[] vertices;
	private ArrayList<Integer> exclude;
	private Translation2d targetPosition;
	private double radius;
	public Pathfinder() {
		faces = new Face[0];
	}

	public Optional<Translation2d> intersectionPoint(Translation2d point0, Translation2d point1, Face face) {
		Translation2d point2 = vertices[face.a];
		Translation2d point3 = vertices[face.b];

		double dx01 = point0.getX() - point1.getX();
		double dy01 = point0.getY() - point1.getY();
		double dy23 = point2.getY() - point3.getY();
		double dx23 = point2.getX() - point3.getX();

		double denominator = dx01 * dy23 - dy01 * dx23;
		if (denominator == 0) return Optional.empty();

		double t = (point0.getX() - point2.getX()) * dy23 - (point0.getY() - point2.getY()) * dx23;
		//Although you will typically check if 0 < t < 1, before we do a division we can 0 < t < denominator
		if (t >= denominator || t <= 0.0) return Optional.empty();

		double u = -(dx01 * (point0.getY() - point2.getY()) - dy01 * (point0.getX() - point2.getX()));
		if (u >= denominator || u <= 0.0) return Optional.empty();

		return Optional.of(point0.plus((point1.minus(point0)).times(t / denominator)));
	}

	/**
	 * @param a One end of the test path
	 * @param b The other end of the test path
	 * @return The index of the intersected face if there was one
	 */
	public Optional<Integer> getIntersection(Translation2d a, Translation2d b) {
		Translation2d delta = b.minus(a);

		/*
		* This, as well as all the other "planes" in this function are not planes tangent to [INSERT FACE HERE];
		* but rather the normal vector that I call a plane. I do this because it helps me visualize the dot product
		* in my head because I'm just spewing this garbage out of my head as it comes.
		*/
		Translation2d plane = new Translation2d(-delta.getY(), delta.getX()).div(delta.getNorm());

		double pathProjection = b.getX() * plane.getX() + b.getY() * plane.getY();

		double lowerProjectionBound = pathProjection - radius;
		double upperProjectionBound = pathProjection + radius;

		double closestDistance = 1e10;
		int closestFace = -1;

		for (int i = 0; i < faces.length; ++i) {
			boolean skip = false;
			if (!exclude.isEmpty()) {
				if (i == exclude.get(exclude.size() - 1)) {
					continue;
				}
			}

			for (Integer integer : exclude) {
				if (i == integer) {
					skip = true;
					if (closestFace == -1) closestFace = -2;
					break;
				}
			}

			if (skip) continue;

			/*
			* Alright, I should probably explain what is going on here...
			* So, I figured, if you want to intersect a sphere's path with a line, you can't just do a simple parametric equivalency.
			* Step one:
			* 	Project the trajectory and the face onto the trajectory's normal plane
			 */

			double lowerFaceProjBound = vertices[faces[i].a].getX() * plane.getX() + vertices[faces[i].a].getY() * plane.getY();
			double upperFaceProjBound = vertices[faces[i].b].getX() * plane.getX() + vertices[faces[i].b].getY() * plane.getY();

			if (lowerFaceProjBound > upperFaceProjBound) {
				double temp = lowerFaceProjBound;
				lowerFaceProjBound = upperFaceProjBound;
				upperFaceProjBound = temp;
			}

			/*
			* Next, check to see if the two ranges of the projections or "shadows" intersect
			 */
			if (lowerProjectionBound <= upperFaceProjBound && lowerFaceProjBound <= upperProjectionBound) {
				/*
				* You may have realized that you can be not intersecting but still have "shadows" intersect.
				* Now, run the same calculation, but with the face's normal as the plane
				 */
				Translation2d faceDelta = vertices[faces[i].b].minus(vertices[faces[i].a]);
				Translation2d facePlane = new Translation2d(-faceDelta.getY(), faceDelta.getX()).div(faceDelta.getNorm());
				double lowerProjectionOnFace = a.getX() * facePlane.getX() + a.getY() * facePlane.getY();
				double upperProjectionOnFace = b.getX() * facePlane.getX() + b.getY() * facePlane.getY();

				if (lowerProjectionOnFace > upperProjectionOnFace) {
					double temp = lowerProjectionOnFace;
					lowerProjectionOnFace = upperProjectionOnFace;
					upperProjectionOnFace = temp;
				}

				lowerProjectionOnFace -= radius;
				upperProjectionOnFace += radius;

				Translation2d vertexCache = vertices[faces[i].a];
				double projectionOfFaceOnFace = vertexCache.getX() * facePlane.getX() + vertexCache.getY() * facePlane.getY(); //Fuck these variable names

				//Anyway, that's basically the function... I put like an hour's worth of brain power into this calculation idea.
				if (lowerProjectionOnFace <= projectionOfFaceOnFace && projectionOfFaceOnFace <= upperProjectionOnFace) {
					double projectionOfAOnFace = a.getX() * facePlane.getX() + a.getY() * facePlane.getY();
					Translation2d pointOnPlane = vertexCache.plus(facePlane.times(projectionOfAOnFace));

					double distance = a.getDistance(pointOnPlane);
					if (distance < closestDistance) {
						closestDistance = distance;
						closestFace = i;
					}
				}
			}
		}

		if (closestFace == -1) {
			return Optional.empty();
		} else if (closestFace == -2) {		//== -2
			return Optional.of(-1);	//return -1 (Smartest Java.Optional user D:)
		} else {
			return Optional.of(closestFace);
		}
	}

	public double getDistance(Translation2d start, Translation2d end) {
		var intersection = getIntersection(start, end);

		if (intersection.isPresent()) {
			int face = intersection.get();
			if (face == -1) {
				return 1e10;
			} else {
				exclude.add(face);

				Translation2d faceDelta = vertices[faces[face].b].minus(vertices[faces[face].a]);
				faceDelta = faceDelta.div(faceDelta.getNorm());

				Translation2d vertexA = vertices[faces[face].a].minus(faceDelta.times(radius));
				Translation2d vertexB = vertices[faces[face].b].plus(faceDelta.times(radius));

				double distanceA = getDistance(vertexA, end);
				double distanceB = getDistance(vertexB, end);

				distanceA += vertexA.getDistance(start);
				distanceB += vertexB.getDistance(start);

				exclude.remove(exclude.size() - 1);

				return Math.min(distanceA, distanceB);
			}
		} else {
			return start.getDistance(end);
		}
	}

	public void setTargetPosition(Translation2d position, double radius) {
		this.targetPosition = position;
		this.radius = radius;
		exclude = new ArrayList<Integer>(faces.length);
	}
	public Translation2d generateWaypoint(Translation2d position) {
		Translation2d delta = targetPosition.minus(position);

		Translation2d plane = new Translation2d(-delta.getY(), delta.getX()).div(delta.getNorm());

		double pathProjection = targetPosition.getX() * plane.getX() + targetPosition.getY() * plane.getY();

		double lowerProjectionBound = pathProjection - radius;
		double upperProjectionBound = pathProjection + radius;

		double closestDistance = 1e10;
		int closestFace = -1;

		for (int i = 0; i < faces.length; ++i) {
			double lowerFaceProjBound = vertices[faces[i].a].getX() * plane.getX() + vertices[faces[i].a].getY() * plane.getY();
			double upperFaceProjBound = vertices[faces[i].b].getX() * plane.getX() + vertices[faces[i].b].getY() * plane.getY();

			if (lowerFaceProjBound > upperFaceProjBound) {
				double temp = lowerFaceProjBound;
				lowerFaceProjBound = upperFaceProjBound;
				upperFaceProjBound = temp;
			}

			if (lowerProjectionBound <= upperFaceProjBound && lowerFaceProjBound <= upperProjectionBound) {
				Translation2d faceDelta = vertices[faces[i].b].minus(vertices[faces[i].a]);
				Translation2d facePlane = new Translation2d(-faceDelta.getY(), faceDelta.getX()).div(faceDelta.getNorm());
				double lowerProjectionOnFace = position.getX() * facePlane.getX() + position.getY() * facePlane.getY();
				double upperProjectionOnFace = targetPosition.getX() * facePlane.getX() + targetPosition.getY() * facePlane.getY();

				if (lowerProjectionOnFace > upperProjectionOnFace) {
					double temp = lowerProjectionOnFace;
					lowerProjectionOnFace = upperProjectionOnFace;
					upperProjectionOnFace = temp;
				}

				lowerProjectionOnFace -= radius;
				upperProjectionOnFace += radius;

				Translation2d vertexCache = vertices[faces[i].a];
				double projectionOfFaceOnFace = vertexCache.getX() * facePlane.getX() + vertexCache.getY() * facePlane.getY(); //Fuck these variable names

				if (lowerProjectionOnFace <= projectionOfFaceOnFace && projectionOfFaceOnFace <= upperProjectionOnFace) {
					//Thank the lord we finally made it!

					double projectionOfAOnFace = position.getX() * facePlane.getX() + position.getY() * facePlane.getY();
					Translation2d pointOnPlane = vertexCache.plus(facePlane.times(projectionOfAOnFace));

					double distance = position.getDistance(pointOnPlane);
					if (distance < closestDistance) {
						closestDistance = distance;
						closestFace = i;
					}
				}
			}
		}

		if (closestFace == -1) {
			return targetPosition;
		} else {
			exclude.add(closestFace);

			Translation2d faceDelta = vertices[faces[closestFace].b].minus(vertices[faces[closestFace].a]);
			faceDelta = faceDelta.div(faceDelta.getNorm());

			Translation2d vertexA = vertices[faces[closestFace].a].minus(faceDelta.times(radius));
			Translation2d vertexB = vertices[faces[closestFace].b].plus(faceDelta.times(radius));

			if (vertexA.getDistance(position) <= 0.0001) {
				return targetPosition;
			}

			if (vertexB.getDistance(position) <= 0.0001) {
				return targetPosition;
			}

			double distanceA = getDistance(vertexA, targetPosition);
			double distanceB = getDistance(vertexB, targetPosition);

			distanceA += vertexA.getDistance(position);
			distanceB += vertexB.getDistance(position);

			exclude.remove(exclude.size() - 1);

			if (distanceA < distanceB) {
				return vertexA;
			} else {
				if (distanceB <= 0.0001) {
					return targetPosition;
				}

				return vertexB;
			}
		}
	}
}
