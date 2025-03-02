package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.OptionalDouble;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class ObjectDetetectorCamera extends SubsystemBase {
  // TODO: fix this mess
  private PhotonCamera m_Camera;
  private PhotonPipelineResult m_Result;
  private List<PhotonTrackedTarget> m_Targets;

  public ObjectDetetectorCamera(String name) {
    m_Camera = new PhotonCamera(name);
  }

  public void updateByUnreadResults() {
    List<PhotonPipelineResult> unreadResults = m_Camera.getAllUnreadResults();
    if (unreadResults.size() > 0) {
      m_Result = unreadResults.get(unreadResults.size() - 1);
    }
  }

  // Unused now
  // Using deprecated method for testing
  // public void updateByLatestResult() {
  //   m_Result = m_Camera.getLatestResult();
  // }

  public OptionalDouble getTargetPitch(int targetID) {
    if (m_Result.hasTargets()) {
      return OptionalDouble.of(m_Result.getTargets().get(targetID).getPitch());
    }
    return OptionalDouble.empty();
  }

  public OptionalDouble getTargetYaw(int targetID) {
    if (m_Result.hasTargets()) {
      return OptionalDouble.of(m_Result.getTargets().get(targetID).getYaw());
    }
    return OptionalDouble.empty();
  }

  public OptionalDouble getTargetSkew(int targetID) {
    if (m_Result.hasTargets()) {
      return OptionalDouble.of(m_Result.getTargets().get(targetID).getSkew());
    }
    return OptionalDouble.empty();
  }

  public OptionalDouble getTargetArea(int targetID) {
    if (m_Result.hasTargets()) {
      return OptionalDouble.of(m_Result.getTargets().get(targetID).getArea());
    }
    return OptionalDouble.empty();
  }

  public OptionalDouble getTargetXFromCenter(int targetID) {
    if (m_Result.hasTargets()) {
      List<TargetCorner> corners = m_Result.getTargets().get(targetID).getMinAreaRectCorners();
      double sum = 0;
      for (TargetCorner corner : corners) {
        sum += corner.x;
      }
      return OptionalDouble.of(320 - sum / 4);
    }
    return OptionalDouble.empty();
  }
}
