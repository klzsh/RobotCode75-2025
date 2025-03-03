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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged(strategy = Strategy.OPT_IN)
public class AprilTagCamera extends SubsystemBase {
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private AprilTagFieldLayout m_tagLayout =
      // FMA uses welded field layout
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private PhotonPoseEstimator m_poseEstimator;
  private EstimatedRobotPose m_pose;
  private Transform3d cameraToRobotPose;

  public AprilTagCamera(String name, Transform3d cameraPose) {
    cameraToRobotPose = cameraPose;
    m_camera = new PhotonCamera(NetworkTableInstance.getDefault(), name);

    m_poseEstimator =
        new PhotonPoseEstimator(m_tagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraPose);
    // m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  public boolean hasTarget() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    return (!targets.isEmpty());
  }

  /** number of targets the camera sees */
  public int numTargets() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    return targets.size();
  }

  /**
   * get the targeted april tag
   *
   * @return ID of the targeted tag
   */
  public OptionalInt getPrimaryTagID() {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    if (m_result.hasTargets()) {
      return OptionalInt.of(targets.get(0).getFiducialId());
    } else {
      return OptionalInt.empty();
    }
  }

  /**
   * returns tag Ids of all tags seen
   *
   * @return
   */
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

  // height in meters of the chosen tag
  public double getAprilTagHeight(int id) {
    return m_tagLayout.getTagPose(id).get().getY();
  }

  public Optional<Pose2d> getMultiTagResult() {
    Optional<MultiTargetPNPResult> target = m_result.getMultiTagResult();
    if (target.isPresent()) {
      return Optional.of(
          new Pose2d(
              target.get().estimatedPose.best.getTranslation().toTranslation2d(),
              target.get().estimatedPose.best.getRotation().toRotation2d()));
    } else {
      return Optional.empty();
    }
  }

  public PoseStrategy getStrategy() {
    return m_poseEstimator.getPrimaryStrategy();
  }

  public Optional<PhotonTrackedTarget> getTarget(int id) {
    List<PhotonTrackedTarget> targets = m_result.getTargets();
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    if (m_result.getBestTarget() == null) {
      return Optional.empty();
    }
    return Optional.of(m_result.getBestTarget());
  }

  public OptionalDouble getRange(int id) {
    PhotonTrackedTarget target = getTarget(id).isPresent() ? getTarget(id).get() : null;
    if (target == null) {
      return OptionalDouble.empty();
    }
    double targetHeight = getAprilTagHeight(id);
    return OptionalDouble.of(
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraToRobotPose.getZ(), targetHeight, 0, Units.degreesToRadians(target.getPitch())));
  }

  public OptionalDouble getX(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getYaw());
    }
  }

  // @Logged
  public double getXSin() {
    if (getTarget(18).isEmpty()) {
      return 0;
    } else {
      PhotonTrackedTarget target = getTarget(18).get();
      return Math.sin(Units.degreesToRadians(target.getYaw()));
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

  // @Logged
  public double getYSin() {
    if (getTarget(18).isEmpty()) {
      return 0;
    } else {
      PhotonTrackedTarget target = getTarget(18).get();
      return Math.sin(Units.degreesToRadians(target.getPitch()));
    }
  }

  public OptionalDouble getSkew(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getSkew());
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

  public OptionalDouble getArea(int id) {
    if (getTarget(id).isEmpty()) {
      return OptionalDouble.empty();
    } else {
      PhotonTrackedTarget target = getTarget(id).get();
      return OptionalDouble.of(target.getArea());
    }
  }

  public OptionalDouble minTagArea() {
    double minArea = Double.MAX_VALUE;
    for (PhotonTrackedTarget target : m_result.getTargets()) {
      if (target.getArea() < minArea) {
        minArea = target.getArea();
      }
    }

    if (minArea == Double.MAX_VALUE) {
      return OptionalDouble.empty();
    } else {
      return OptionalDouble.of(minArea);
    }
  }

  // @Logged
  public double getPrimaryTagX() {
    if (m_result.getTargets().size() >= 1) {
      return m_result.getBestTarget().getYaw();
    } else {
      return -1;
    }
  }

  // @Logged
  public double getPrimaryTagY() {
    if (m_result.getTargets().size() >= 1) {
      return m_result.getBestTarget().getPitch();
    } else {
      return -1;
    }
  }

  // @Logged
  public double getPrimaryTagTheta() {
    if (m_result.getTargets().size() >= 1) {
      return m_result.getBestTarget().getSkew();
    } else {
      return -1;
    }
  }

  public EstimatedRobotPose getEstimatedPose() {
    if (minTagArea().isPresent() && minTagArea().getAsDouble() < 0) {
      return null;
    }
    return m_pose;
  }

  @Logged(name = "Estimated Pose", importance = Importance.DEBUG)
  public Pose2d getEstimatedPose2d() {
    if (m_pose != null) {
      return m_pose.estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  /**
   * time the frame was taken
   *
   * @return timestamp in seconds
   */
  public double getTimestamp() {
    return m_result.getTimestampSeconds();
  }

  public void updatePoseEstimator() {
    if (m_poseEstimator != null) {
      Optional<EstimatedRobotPose> pose;
      List<PhotonPipelineResult> m_unreadResults;
      m_unreadResults = m_camera.getAllUnreadResults();
      if (!m_unreadResults.isEmpty()) {
        // gets the latest unread result
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

  private Pose3d getTagPose(int id) {
    return m_tagLayout.getTagPose(id).get();
  }

  /**
   * Gets the 3d pose of the april tags that the camera sees. Used mainly for AdvantageScope
   * Visualization
   *
   * @return poses of seen april tags
   */
  // @Logged(name = "Seen Tag Poses", importance = Importance.DEBUG)
  public Pose3d[] getSeenTags() {
    List<Pose3d> targets = new ArrayList<>();
    if (getAllTagIds().isPresent()) {
      for (Integer tag : getAllTagIds().get()) {
        targets.add(getTagPose(tag));
      }
    }
    return targets.toArray(new Pose3d[targets.size()]);
  }

  /**
   * gets the estimated robot pose from the camera
   *
   * @return
   */
  // @Logged(name = "Camera Estimated Pose", importance = Importance.DEBUG)
  public Pose2d getVisionPose() {
    if (m_pose != null) {
      return m_pose.estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  public void updateHeading(Rotation2d heading) {
    m_poseEstimator.addHeadingData(Timer.getFPGATimestamp(), heading);
  }

  // public double getLatency() {  // This dont exist in 2025 either ig
  //   return m_result.getLatencyMillis() / 1000.0;
  // }

  @Override
  public void periodic() {
    updatePoseEstimator();
  }
}
