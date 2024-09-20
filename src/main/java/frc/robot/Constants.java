// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

import java.io.IOException;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public final static String CANBUS_NAME = "drivebase";
	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
	public static final AprilTagFieldLayout aprilTagFieldLayout;

	static {
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}



	public static class OperatorConstants {

		// Joystick Deadband
		public static final double LEFT_X_DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
	}

	public static class DriverConstants {

		// Joystick Deadband
		public static final double LEFT_X_DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
	}

	public static final class Vision {
		public static final String intakeColorCam = "4kcam1";
		public static final String shooterMonoCam = "Arducam_OV9281_Krog";
		public static final Transform3d shooterRobotToCamera = new Transform3d(new Translation3d(0.175, -0.175, 0.600), new Rotation3d(0,Math.toRadians(-23),Math.toRadians(180)));
		public static final Transform3d lowerRobotToCamera = new Transform3d(new Translation3d(-0.152, 0, 0.267), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180)));
	}

	public static final class Shooter {
		public static final int launchKrakenID = 12;
		public static final int angleFalconID = 11;
		public static final int angleCANCoderID = 5;
		public static final int neo550LowGearID = 3;
		public static final int neo550HighGearID = 5;
		public static final int neo55HighGearRatio = 4;
		public static final int neo550LowGearRatio = 1;
		/**
		 * Velocity Setpoint for the Shooter
		 */
		public static final double shooterVelSetpoint = 64;
		/**
		 * Difference from setpoint at which indexing motor runs for shooterCommand
		 */
		public static final double shooterVelTolerance = 5;

		//12T to 42T, 15T to 42T gears
		public static final double angleGearRatio = 180d / 1764d;
		public static final double angleCancoderGearRatio = 15d / 42d;
		public static final double angleFrontHardstop = -20;
		public static final double angleBackHardstop = 54.58;
//		public static final PIDConstants anglePIDConstants = new PIDConstants(1,2.5,0);
		public static final double angleFeedForward = 0.6;
		public static final double angleCanCoderZero = 0.086;
		public static final double angleRotorToSensorGearRatio = 12d / 42d;

	}
	public static final class Intake {
		public static final int lowerNeoID = 13;
		public static final int upperNeoID = 12;
		public static final double lowerIntakeSpeed = 0.75;
		public static final double upperIntakeSpeed = 0.75;
	}
	public static final class Climb {
		public static final int upperFalconID = 9;
		public static final int lowerKrakenID = 10;
		public static final double climbSpeed = 0.4;

		public static final double hoistUpPosition = 0.0;
		public static final double hoistDownPosition = 0.0;
		public static final double currentLimit = 80;

		public static final double kP = 0.1;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kS = 0.2;

		/**
		 * Upper Kraken Motor Rotations for Climb Amp Position Control
		 */
		public static final double climbAmpPosition = -177;
	}

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled

		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static final class AutonConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
	}
}
