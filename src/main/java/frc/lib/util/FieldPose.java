package frc.lib.util;

public class FieldPose {

  public enum Side {
    BLUE,
    RED
  }

  public enum FieldElement {
    TOPHPSTATION,
    BOTTOMHPSTATION,
    REEFA,
    REEFB,
    REEFC,
    REEFD,
    REEFE,
    REEFF,
    REEFG,
    PROCESSOR
  }

  public static boolean fieldElementIsReef(FieldElement fieldElement) {
    return (fieldElement == FieldElement.REEFA
        || fieldElement == FieldElement.REEFB
        || fieldElement == FieldElement.REEFC
        || fieldElement == FieldElement.REEFD
        || fieldElement == FieldElement.REEFE
        || fieldElement == FieldElement.REEFF
        || fieldElement == FieldElement.REEFG);
  }

  public static boolean fieldElementIsHPStation(FieldElement fieldElement) {
    return (fieldElement == FieldElement.TOPHPSTATION
        || fieldElement == FieldElement.BOTTOMHPSTATION);
  }

  public enum Offset {
    NONE,
    LEFT, // used for side loading and for coral intake
    RIGHT, // used for side loading and for coral intake
    SIDELOADMID, // stupid hack
    // left 1 means one inset left from the middlemost one across all 3
    // same thing for right
    // used for front loading HP station
    // 3 and 4 may not be possible depening on side, etc
    LEFT1,
    LEFT2,
    LEFT3,
    LEFT4,
    RIGHT1,
    RIGHT2,
    RIGHT3,
    RIGHT4
  }

  public Side side;
  public FieldElement fieldElement;
  public Offset offset;

  public FieldPose(Side side, FieldElement fieldElement, Offset offset) {
    this.side = side;
    this.fieldElement = fieldElement;
    this.offset = offset;
  }
}
