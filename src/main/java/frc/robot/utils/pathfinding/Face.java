package frc.robot.utils.pathfinding;

public class Face {
	public int a, b; //Indices into a vertex array

	public Face() {
		this.a = -1;
		this.b = -1;
	}
	public Face(int a, int b) {
		this.a = a;
		this.b = b;
	}
}