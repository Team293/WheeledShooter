/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpikeEncoder;
import edu.wpi.first.wpilibj.Talon;

/**
 *
 * @author Robotics Laptop B
 */
public class PShooter {

    Talon motor;
    SpikeEncoder encoder;
    double kP, tolerance, setpoint;

    /**
     *
     * @param motorPort
     * @param encoderA
     * @param encoderB
     * @param kP
     * @param tolerance
     * @param setpoint
     */
    PShooter(int motorPort, int encoderA, int encoderB, double kP, double tolerance, double setpoint) {
        motor = new Talon(motorPort);
        encoder = new SpikeEncoder(encoderA, encoderB);
        this.kP = kP;
        this.tolerance = tolerance;
        this.setpoint = setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enable() {
        encoder.start();
        encoder.reset();
    }

    public void run() {
        double output = kP * (setpoint - encoder.getRPM());
        motor.set(output);
    }

    public void setConstant(double kP) {
        this.kP = kP;
    }
}
