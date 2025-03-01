package frc.lib.dashboard;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;

/*
 * Takes in a string to parse into an auto
 * Enter Command - place to type/paste in the string
 * Feedback - displays errors, shows completed path
 * Generate - checks paths for errors, displays trajectories on field, sets auto command
 * Reset - reset trajectories, string, and auto command
 *
 * FORMAT FOR AUTO STRING
 * Separated into words by spaces - commands in the same word are executed simultaneously
 * Each word can contain one point and one action max
 *  - All points are strings of letters (case insensitive)
 *  - All actions are numbers
 * !!! The first word MUST be only one starting point (ST/SM/SB) without an action
 * Points are labeled according to position in Choreo/relative to processor
 */

/* ACTIONS
 * 1 for align + score L1 + move back to R point
 * 2 for align + elevator movement & dealgify (decide level by last point or apriltag) + move back to R point
 * 3, 4 for align + elevator movement & score left/right L4 + move back to R point
 * 5 for align + elevator movement & processor + move back to P point
 * 6, 7, 8 for align to left/middle/right coral station + check for beam break + move back to H point
 */

/* POINTS
 * ST, SM, SB - top/middle/bottom starting positions
 * L - arbitrary leave point
 * P - processor
 * RTL, RTR, RBL, RBR, RR, RL - reef points
 * HT, HB - human player stations
 */

public class AutoSelector {

  private class ChoreoTrajectory {
    public choreo.trajectory.Trajectory traj;

    public ChoreoTrajectory(choreo.trajectory.Trajectory t) {
      traj = t;
    }
  }

  private List<ChoreoTrajectory> m_trajectories = new ArrayList<>();
  private Command m_autoCommand = Commands.runOnce(() -> {});
  private Pose2d m_startPose;
  private Field2d m_field;
  private Map<Integer, Command> m_actionMap;
  private Swerve m_swerve;
  private List<Command> m_startCommands;
  private List<Command> m_endCommands;
  private Map<String, Pose2d> m_startPositions;

  private GenericEntry autoStringEntry;
  private GenericEntry feedbackEntry;
  // private GenericEntry safetyEntry;

  private final AutoFactory factory;

  public AutoSelector(
      Map<Integer, Command> actionMap,
      Swerve swerve,
      List<Command> startCommands,
      List<Command> endCommands) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = nt.getTable("Shuffleboard").getSubTable("Auto");
    autoStringEntry = table.getTopic("Enter Command").getGenericEntry();
    feedbackEntry = table.getTopic("Feedback").getGenericEntry();
    feedbackEntry.setString("Enter a command!");

    m_field = new Field2d();
    m_actionMap = actionMap;
    m_swerve = swerve;
    m_startCommands = startCommands;
    m_endCommands = endCommands;

    // define auto factory for autos
    factory =
        new AutoFactory(
            m_swerve::getPose,
            m_swerve::setPose,
            m_swerve::followSwerveSample,
            true,
            m_swerve);
  }

  public void clearField() {
    for (int i = 0; i < 100; i++) {
      FieldObject2d obj = m_field.getObject("traj" + i);
      obj.setTrajectory(new edu.wpi.first.math.trajectory.Trajectory());
    }
  }

  private void drawPaths() {
    clearField();
    for (int i = 0; i < m_trajectories.size(); i++) {
      ChoreoTrajectory pathTraj = m_trajectories.get(i);
      List<Pose2d> poses = Arrays.asList(pathTraj.traj.getPoses());
      Trajectory displayTraj =
          TrajectoryGenerator.generateTrajectory(
              poses, new TrajectoryConfig(AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration));
      m_field.getObject("traj" + i).setTrajectory(displayTraj);
    }
  }

  public void clearAll() {
    m_trajectories.clear();
    clearField();
  }

  public void reset() {
    clearAll();
    autoStringEntry.setString("");
    feedbackEntry.setString("Enter a command!");
    m_autoCommand = Commands.runOnce(() -> {});
    m_startPose = null;
  }

  public void setFeedback(String feedback) {
    feedbackEntry.setString(feedback);
  }

  public String getFeedback() {
    return feedbackEntry.getString("");
  }

  public void setupAutoTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    autoTab.add("Enter Command", "").withSize(4, 1).withPosition(0, 0);
    autoTab.add(m_field).withSize(6, 4).withPosition(4, 0);
    autoTab.addString("Feedback", () -> getFeedback()).withSize(4, 1).withPosition(0, 1);

    autoTab
        .add("Generate", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1, 1)
        .withPosition(0, 2);
    autoTab
        .add("Reset", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(1, 1)
        .withPosition(1, 2);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable ntTable = inst.getTable("Shuffleboard").getSubTable("Auto");

    ntTable.addListener(
        "Generate",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
          generatePaths();
        });
    ntTable.addListener(
        "Reset",
        EnumSet.of(Kind.kValueAll),
        (table, key, event) -> {
          reset();
        });

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      m_startPositions = AutoConstants.blueStartPositions;
    } else {
      m_startPositions = AutoConstants.redStartPositions;
    }
  }

  public void generatePaths() {
    String autoString = autoStringEntry.getString("");
    String[] words = autoString.split(" ");

    if (!m_startPositions.containsKey(words[0].toLowerCase())) {
      setFeedback("Invalid start position");
      return;
    }
    m_startPose = m_startPositions.get(words[0].toLowerCase());

    SequentialCommandGroup sequential = new SequentialCommandGroup();
    StringBuilder s = new StringBuilder();
    m_trajectories.clear();

    for (Command startCommand : m_startCommands) {
      sequential.addCommands(startCommand);
      s.append(startCommand.getName() + " ");
    }

    if (autoString.length() == 0) {
      for (Command endCommand : m_endCommands) {
        sequential.addCommands(endCommand);
      }
      m_autoCommand = sequential;
      setFeedback("Empty path. Is this intentional?");
      return;
    }

    String lastPose = "";
    for (int i = 0; i < words.length; i++) {
      ParallelCommandGroup parallelGroup = new ParallelCommandGroup();
      StringBuilder pointString = new StringBuilder();
      StringBuilder actionString = new StringBuilder();
      for (int j = 0; j < words[i].length(); j++) {
        char c = words[i].charAt(j);
        if (Character.isLetter(c)) {
          pointString.append(c);
        } else {
          actionString.append(c);
        }
      }

      String point = pointString.toString().toLowerCase();
      int action = actionString.length() > 0 ? Integer.parseInt(actionString.toString()) : -1;

      if (i == 0) {
        lastPose = point;
        continue;
      }
      if (point != "" && lastPose != "") {
        try {
          // m_trajectories.add(
          //     new ChoreoTrajectory(Choreo.loadTrajectory("" + lastPose + "-" + point).get()));
          parallelGroup.addCommands(factory.trajectoryCmd("" + lastPose + "-" + point));
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            m_trajectories.set(
                m_trajectories.size() - 1,
                new ChoreoTrajectory(m_trajectories.get(m_trajectories.size() - 1).traj.flipped()));
          }
          s.append("" + lastPose + "-" + point + " ");
          lastPose = point;
        } catch (Exception e) {
          setFeedback("Path File Not Found: " + lastPose + "-" + point);
          m_autoCommand = Commands.runOnce(() -> {});
          return;
        }
      }
      if (m_actionMap.containsKey(action)) {
        parallelGroup.addCommands(m_actionMap.get(action));
        s.append(m_actionMap.get(action).getName() + " ");
      } else if (action != -1) {
        setFeedback("Action Not Found: " + action);
        m_autoCommand = Commands.runOnce(() -> {});
        return;
      }
      sequential.addCommands(parallelGroup);
      sequential.addCommands(new InstantCommand(() -> m_swerve.stopModules(), m_swerve));
    }
    setFeedback("Auto Sequence: " + s.toString());
    drawPaths();
    m_autoCommand = sequential;
  }

  public Command getAutoCommand() {
    return m_autoCommand;
  }

  public Pose2d getStartPose() {
    return m_startPose;
  }
}
