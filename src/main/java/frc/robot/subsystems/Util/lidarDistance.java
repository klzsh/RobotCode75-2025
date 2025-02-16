// package frc.robot.subsystems.Util;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.DigitalInput;



// public class lidarDistance extends SubsystemBase {

//     private double coralL1ThresholdIN = 24;

//     public lidarDistance() {
//         DigitalInput dioInput = new DigitalInput(0); // DIO Port 0
//         DutyCycle sensorCycle = new DutyCycle(dioInput);
//     }

//     private double getTimeNanoSeconds() {
//         return sensorCycle.getHighTimeNanoSeconds();
//     }

//     private double getDistanceMM() {
//         return 4*(getTimeNanoSeconds() - 1e6);
//     }

//     private double getDistanceIN() {
//         return getDistanceMM()*0.0393701;
//     }

//     private boolean getAligned() {
//         return getDistanceIN() <= coralL1Threshold;
//     }
// }