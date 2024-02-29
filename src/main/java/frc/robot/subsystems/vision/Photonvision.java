package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;

public class Photonvision extends SubsystemBase {
	private PhotonPipelineResult pipelineResult;
	private PhotonPoseEstimator poseEstimator;
	private final PhotonCamera camera;

	/**
	 * A subsystem to initialize {@link Photonvision} camera instances to allow for multiple camera pose estimation and measurements.
	 * @param cameraName String, the camera name, as it shows on the PhotonVision dashboard
	 * @param cameraToRobot Transform3D, where the camera is relative to the robot
	 */
	public Photonvision(String cameraName, Transform3d cameraToRobot) {
		this.camera = new PhotonCamera(cameraName);
		pipelineResult = new PhotonPipelineResult();

		try {
			AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
			poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
		} catch(IOException e) {
			DriverStation.reportError(e.toString(), true);
		}
	}


	/**
	 * Get the PhotonCamera instance
	 * @return PhotonCamera
	 */
	public PhotonCamera getCamera() {
		return camera;
	}

	/**
	 * Get the Photon Pose Estimator from camera
	 * @return PhotonPoseEstimator Instance
	 */
	public PhotonPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	/**
	 *
	 * @return Boolean, if there's a target or not
	 */
	public boolean hasTargets() {
		return pipelineResult.hasTargets();
	}

	/**
	 *
	 * @return A list of all PhotonPipelineResults targets
	 */
	public List<PhotonTrackedTarget> targets() {
		return pipelineResult.targets;
	}

	/**
	 *
	 * @return Returns the best target in PhotonPipelineResult
	 */
	public PhotonTrackedTarget getBestTarget() {
		return pipelineResult.getBestTarget();
	}

	@Override
	public void periodic() {
		pipelineResult = camera.getLatestResult();
	}

}