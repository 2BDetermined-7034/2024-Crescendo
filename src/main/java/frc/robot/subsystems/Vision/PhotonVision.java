package frc.robot.subsystems.Vision;

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

public class PhotonVision extends SubsystemBase {
    private PhotonPipelineResult pipelineResult;
    private PhotonPoseEstimator poseEstimator;
    private final PhotonCamera camera;

    public PhotonVision(String cameraName, Transform3d cameraToRobot) {
        this.camera = new PhotonCamera(cameraName);
        pipelineResult = new PhotonPipelineResult();

        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
        } catch(IOException e) {
            DriverStation.reportError(e.toString(), true);
        }
    }


    public PhotonCamera getCamera() {
        return camera;
    }


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

    public List<PhotonTrackedTarget> targets() {
        return pipelineResult.targets;
    }

    public PhotonTrackedTarget getBestTarget() {
        return pipelineResult.getBestTarget();
    }

    @Override
    public void periodic() {
        pipelineResult = camera.getLatestResult();
    }

}