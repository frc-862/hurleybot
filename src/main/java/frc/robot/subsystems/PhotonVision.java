// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult result;

    private Pose4d estimatedRobotPose = new Pose4d();
    private Field2d field = new Field2d();

    private double lastPoseTime = 0;

    public PhotonVision() {
        camera = new PhotonCamera("camz");

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update(result);
    }

    public void setEstimatedPose(EstimatedRobotPose pose) {
        estimatedRobotPose = new Pose4d(pose.estimatedPose.getTranslation(), pose.estimatedPose.getRotation(), pose.timestampSeconds - lastPoseTime);

        lastPoseTime = pose.timestampSeconds;
    }


    @Override
    public void periodic() {
        try {
            result = camera.getLatestResult();

        } catch (IndexOutOfBoundsException e) {
            System.out.println("no camera data recieved");
        }


        if (result != null && poseEstimator != null) {
            
            poseEstimator.update(result).ifPresent((m_estimatedRobotPose) -> setEstimatedPose(m_estimatedRobotPose));


            field.setRobotPose(estimatedRobotPose.toPose2d());
            
            LightningShuffleboard.set("Vision", "pose", field);

        } else {
            System.out.println("the other no camera data recieved");

        }
    }
}
