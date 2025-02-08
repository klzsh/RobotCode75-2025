// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged(strategy = Strategy.OPT_IN)
public class AprilTagCamera extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private AprilTagFieldLayout m_tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private PhotonPoseEstimator m_poseEstimator;
  private EstimatedRobotPose m_pose;

  public AprilTagCamera(String name, Transform3d cameraPose) {
    m_camera = new PhotonCamera(NetworkTableInstance.getDefault(), name);

    m_poseEstimator =
        new PhotonPoseEstimator(m_tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPose);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public boolean hasTarget() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();

    return (!targets.isEmpty());
  }

  public int numTargets() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    return targets.size();
  }

  public OptionalInt getPrimaryTagID() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    if (m_result.hasTargets()) {
      return OptionalInt.of(targets.get(0).getFiducialId());
    } else {
      return OptionalInt.empty();
    }
  }

  // TODO: null check for tag ids
  public Optional<List<Integer>> getAllTagIds() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    ArrayList<Integer> ids = new ArrayList<Integer>();

    for (PhotonTrackedTarget target : targets) {
      ids.add(target.getFiducialId());
    }
    if (ids.isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(ids);
    }
  }

  public double getAprilTagHeight(int id) {
    return m_tagLayout.getTagPose(id).get().getY();
  }

  // public Optional<Pose2d> getMultiTagResult() {  // This doesn't work in 2025 ig
  //   Optional<MultiTargetPNPResult> target = m_result.getMultiTagResult();
  //   if (target.estimatedPose.isPresent) {
  //     return Optional.of(
  //         new Pose2d(
  //             target.estimatedPose.best.getTranslation().toTranslation2d(),
  //             target.estimatedPose.best.getRotation().toRotation2d()));
  //   } else {
  //     return Optional.empty();
  //   }
  // }

  public PoseStrategy getStrategy() {
    return m_poseEstimator.getPrimaryStrategy();
  }

  // todo: null check on getting target
  public Optional<PhotonTrackedTarget> getTarget(int id) {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  // public OptionalDouble getRange(int id) {
  //   PhotonTrackedTarget target = getTarget(id);
  //   if (target == null) {
  //     return OptionalDouble.empty();
  //   }
  //   double targetHeight = getAprilTagHeight(id);
  //   return OptionalDouble.of(
  //       PhotonUtils.calculateDistanceToTargetMeters(
  //           kCameraHeight,
  //           targetHeight,
  //           kCameraPitch,
  //           Units.degreesToRadians(target.getPitch())));
  // }

  public OptionalDouble getX(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getYaw());
    }
  }

  public OptionalDouble getY(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getPitch());
    }
  }

  public OptionalDouble getYaw(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getYaw());
    }
  }

  public OptionalDouble getPitch(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getPitch());
    }
  }

  public EstimatedRobotPose getEstimatedPose() {
    return m_pose;
  }

  public double getTimestamp() {
    return m_result.getTimestampSeconds();
  }

  public void updatePoseEstimator() {
    if (m_poseEstimator != null) {
      Optional<EstimatedRobotPose> pose;
      List<PhotonPipelineResult> m_unreadResults;
      m_unreadResults = m_camera.getAllUnreadResults();
      if (!m_unreadResults.isEmpty()) {
        m_result = m_unreadResults.get(m_unreadResults.size() - 1);
        pose = m_poseEstimator.update(m_result);
      } else { // Latest result is a duplicate
        pose = Optional.empty();
      }
      if (pose.isPresent()) {
        m_pose = pose.get();
      } else {
        m_pose = null;
      }
    }
  }
  private Pose3d getTagPose(int id){
    return m_tagLayout.getTagPose(id).get();
  }

  @Logged(name = "Vision Targets", importance = Importance.INFO)
  public Pose3d[] getPoses() {
    List<Pose3d> targets = new ArrayList<>();
    if(getAllTagIds().isPresent()){
      for(Integer tag: getAllTagIds().get()) {
        targets.add(getTagPose(tag));
      }
    }
    return targets.toArray(new Pose3d[targets.size()]);
  }
  @Logged(name = "Estimated Pose", importance = Importance.INFO)
  public Pose2d getVisionPose(){
    if(m_pose != null){
      return m_pose.estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  // public double getLatency() {  // This dont exist in 2025 either ig
  //   return m_result.getLatencyMillis() / 1000.0;
  // }

  @Override
  public void periodic() {
    updatePoseEstimator();
  }
}
