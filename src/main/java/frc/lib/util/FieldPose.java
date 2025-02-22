package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPose {

  public enum FieldElement {
    HT,
    HB,
    RL,
    RBL,
    RBR,
    RR,
    RTR,
    RTL,
    P,
    // only placeholders, not  used for alignment
    BT,
    BB
  }

  public static FieldElement fromString(String reefPoint) {
    switch (reefPoint.toUpperCase()) {
      case "RL":
        return FieldElement.RL;
      case "RBL":
        return FieldElement.RBL;
      case "RBR":
        return FieldElement.RBR;
      case "RR":
        return FieldElement.RR;
      case "RTR":
        return FieldElement.RTR;
      case "RTL":
        return FieldElement.RTL;
      case "HT":
        return FieldElement.HT;
      case "HB":
        return FieldElement.HB;
      case "P":
        return FieldElement.P;
      case "BT":
        return FieldElement.BT;
      case "BB":
        return FieldElement.BB;
      default:
        return null;
    }
  }

  public static boolean fieldElementIsReef(FieldElement fieldElement) {
    return (fieldElement == FieldElement.RL
        || fieldElement == FieldElement.RBL
        || fieldElement == FieldElement.RBR
        || fieldElement == FieldElement.RR
        || fieldElement == FieldElement.RTR
        || fieldElement == FieldElement.RTL);
  }

  public static boolean fieldElementIsHPStation(FieldElement fieldElement) {
    return (fieldElement == FieldElement.HT || fieldElement == FieldElement.HB);
  }

  public enum Offset {
    LEFT,
    MID,
    RIGHT
  }

  public Alliance alliance;
  public FieldElement fieldElement;
  public Offset offset;

  public FieldPose(Alliance alliance, FieldElement fieldElement, Offset offset) {
    this.alliance = alliance;
    this.fieldElement = fieldElement;
    this.offset = offset;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof FieldPose) {
      FieldPose other = (FieldPose) obj;
      return this.alliance == other.alliance
          && this.fieldElement == other.fieldElement
          && this.offset == other.offset;
    }
    return false;
  }

  @Override
  public String toString() {
    return "FieldPose [alliance="
        + alliance
        + ", fieldElement="
        + fieldElement
        + ", offset="
        + offset
        + "]";
  }
}
