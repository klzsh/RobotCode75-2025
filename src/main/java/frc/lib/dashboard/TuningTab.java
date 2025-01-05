// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.dashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.HardwareConstants;
import java.util.EnumSet;
import java.util.HashMap;

/** Add your docs here. */
public class TuningTab {

  static int col = 0;
  private static HashMap<String, TalonFX> motors;
  private static HashMap<String, PIDController> controllers;
  private static ShuffleboardTab tab;

  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public TuningTab() {
    tab = Shuffleboard.getTab("Tuning");

    /* NOTE: motors is null because addPIDTuner is never called and therefore nothing is put into
    the HashMap */
  }

  public static void addAutoPIDTuner(String name, PIDController pidController) {
    controllers.put(name, pidController);
    NetworkTable ntTable = inst.getTable("Tuning");
    ShuffleboardLayout newLayout = tab.getLayout(name).withSize(2, 4);
    newLayout.add("P", pidController.getP());
    newLayout.add("I", pidController.getI());
    newLayout.add("D", pidController.getD());

    DoubleSubscriber psub = ntTable.getDoubleTopic("P").subscribe(0.0);
    DoubleSubscriber isub = ntTable.getDoubleTopic("I").subscribe(0.0);
    DoubleSubscriber dsub = ntTable.getDoubleTopic("D").subscribe(0.0);

    /* Listener for P */
    inst.addListener(
        psub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          pidController.setP(event.valueData.value.getDouble());
        });

    /* Listener for I */
    inst.addListener(
        isub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          pidController.setI(event.valueData.value.getDouble());
        });

    /* Listener for D */
    inst.addListener(
        dsub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          pidController.setD(event.valueData.value.getDouble());
        });
  }

  public static void addPIDTuner(String name, TalonFX motor) {
    motors.put(name, motor);
    NetworkTable ntTable = inst.getTable("Tuning");
    ShuffleboardLayout newLayout = tab.getLayout(name).withSize(2, 4);
    Slot0Configs config = new Slot0Configs();

    /* Set the config equal to the values in HardwareConstants if empty */
    if (config.kP == 0.0 && config.kI == 0.0 && config.kD == 0.0 && config.kS == 0.0) {
      config.kP = HardwareConstants.Swerve.driveTorqueKP;
      config.kI = HardwareConstants.Swerve.driveTorqueKI;
      config.kD = HardwareConstants.Swerve.driveTorqueKD;
      config.kS = HardwareConstants.Swerve.driveTorqueKS;
    }

    newLayout.add("P", config.kP);
    newLayout.add("I", config.kI);
    newLayout.add("D", config.kD);
    newLayout.add("S", config.kS);

    DoubleSubscriber psub = ntTable.getDoubleTopic("P").subscribe(0.0);
    DoubleSubscriber isub = ntTable.getDoubleTopic("I").subscribe(0.0);
    DoubleSubscriber dsub = ntTable.getDoubleTopic("D").subscribe(0.0);
    DoubleSubscriber ssub = ntTable.getDoubleTopic("S").subscribe(0.0);

    /* Listener for P */
    inst.addListener(
        psub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          config.kP = event.valueData.value.getDouble();
        });

    /* Listener for I */
    inst.addListener(
        isub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          config.kI = event.valueData.value.getDouble();
        });

    /* Listener for D */
    inst.addListener(
        dsub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          config.kD = event.valueData.value.getDouble();
        });

    /* Listener for S */
    inst.addListener(
        ssub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          config.kS = event.valueData.value.getDouble();
        });
  }

  public void setPIDToMotor(Slot0Configs config, TalonFX motor) {
    motor.getConfigurator().apply(config);
  }
}
