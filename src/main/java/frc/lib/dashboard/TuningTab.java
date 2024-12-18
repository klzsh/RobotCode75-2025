// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.KrakenX60;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class TuningTab {

    private class PIDEntry {
        GenericEntry p;
        GenericEntry i;
        GenericEntry d;
        GenericEntry f;
    }

    static int col = 0;
    private static HashMap<String, TalonFX> motors;
    private static HashMap<String, PIDController> controllers;
    private static HashMap<String, ShuffleboardLayout> layouts;
    private static HashMap<String, PIDEntry> pidEntriie
    private static ShuffleboardTab tab;

    public TuningTab() {
        tab = Shuffleboard.getTab("Tuning");
    }

    public static void addPIDTuner(String name, PIDController pidController) {
        controllers.put(name, pidController);
        ShuffleboardLayout newLayout = tab.getLayout(name).withSize(2, 4);
        newLayout.add("P", pidController.getP()).;
    }

    public static void addPIDTuner(String name, TalonFX motor) {
        motors.put(name, motor);

    }


}
