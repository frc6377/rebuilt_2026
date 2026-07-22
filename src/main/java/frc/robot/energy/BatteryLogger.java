// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/** Class for logging current, power, and energy usage. */
public class BatteryLogger {
    private double totalCurrent = 0.0;
    private double controlledCurrent = 0.0;
    private double totalPower = 0.0;
    private double totalEnergy = 0.0;

    private double batteryVoltage = 12.6;
    private double rioCurrent = 0.0;

    public double getControlledCurrent() {
        return controlledCurrent;
    }

    public double getUncontrolledCurrent() {
        return Math.max(0.0, totalCurrent - controlledCurrent);
    }

    public void setBatteryVoltage(double batteryVoltage) {
        this.batteryVoltage = batteryVoltage;
    }

    public void setRioCurrent(double rioCurrent) {
        this.rioCurrent = rioCurrent;
    }

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void reportCurrentUsage(String key, boolean controlled, double... amps) {
        double totalAmps = 0.0;
        for (double amp : amps) {
            totalAmps += Math.abs(amp);
        }
        if (controlled) {
            controlledCurrent += totalAmps;
        }

        double power = totalAmps * batteryVoltage;
        double energy = power * LoggedRobot.defaultPeriodSecs;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsytemCurrents.put(key, totalAmps);
        subsytemPowers.put(key, power);
        subsytemEnergies.merge(key, energy, Double::sum);

        String[] keys = key.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsytemCurrents.merge(subkey, totalAmps, Double::sum);
            subsytemPowers.merge(subkey, power, Double::sum);
            subsytemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    public void periodicAfterScheduler() {
        reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
        reportCurrentUsage("Controls/CANcoders", false, 0.05 * 4);
        reportCurrentUsage("Controls/Pigeon", false, 0.04);
        reportCurrentUsage("Controls/CANivore", false, 0.03);
        reportCurrentUsage("Controls/Radio", false, 0.5);

        Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
        Logger.recordOutput("EnergyLogger/ControlledCurrent", controlledCurrent, "amps");
        Logger.recordOutput("EnergyLogger/UncontrolledCurrent", getUncontrolledCurrent(), "amps");
        Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
        Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

        for (var entry : subsytemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
            subsytemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
            subsytemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemEnergies.entrySet()) {
            Logger.recordOutput(
                    "EnergyLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()), "watt hours");
        }
    }

    public void resetTotals() {
        totalPower = 0.0;
        totalCurrent = 0.0;
        controlledCurrent = 0.0;
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    public double getTotalPower() {
        return totalPower;
    }

    public double getTotalEnergy() {
        return totalEnergy;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}
