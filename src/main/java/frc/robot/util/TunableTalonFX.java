// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A TalonFX with tunable PID gains via NetworkTables. Extends TalonFX so you can use it as a drop-in replacement.
 *
 * <p>Usage examples:
 *
 * <pre>{@code
 * // Create with device ID, CAN bus, tuning name prefix + initial gains
 * TunableTalonFX shooter = new TunableTalonFX(1, "canivore", "Shooter", 0.1, 0.0, 0.0, 0.12, 0.0);
 *
 * // Create from existing Slot0Configs
 * Slot0Configs gains = new Slot0Configs().withKP(0.1).withKV(0.12);
 * TunableTalonFX hood = new TunableTalonFX(2, "rio", "Hood", gains);
 *
 * // In periodic():
 * shooter.updateTunableGains(); // Call to apply any dashboard changes
 * }</pre>
 */
public class TunableTalonFX extends TalonFX {
    private final String tunableName;

    // Tunable PID gains
    private final LoggedNetworkNumber tunableKP;
    private final LoggedNetworkNumber tunableKI;
    private final LoggedNetworkNumber tunableKD;
    private final LoggedNetworkNumber tunableKV;
    private final LoggedNetworkNumber tunableKS;
    private final LoggedNetworkNumber tunableKA;
    private final LoggedNetworkNumber tunableKG;

    // Cache previous values to detect changes
    private double lastKP;
    private double lastKI;
    private double lastKD;
    private double lastKV;
    private double lastKS;
    private double lastKA;
    private double lastKG;

    /**
     * Creates a TunableTalonFX on a specific CAN bus from a Slot0Configs object.
     *
     * @param deviceId The CAN ID of the device
     * @param canbus The name of the CAN bus (e.g., "rio", "canivore")
     * @param tunableName The name prefix for NetworkTables entries
     * @param initialGains The initial Slot0Configs to use for gains
     */
    public TunableTalonFX(int deviceId, String canbus, String tunableName, Slot0Configs initialGains) {
        super(deviceId, new CANBus(canbus));
        this.tunableName = tunableName;

        // Create tunable values from Slot0Configs
        this.tunableKP = new LoggedNetworkNumber(tunableName + "/kP", initialGains.kP);
        this.tunableKI = new LoggedNetworkNumber(tunableName + "/kI", initialGains.kI);
        this.tunableKD = new LoggedNetworkNumber(tunableName + "/kD", initialGains.kD);
        this.tunableKV = new LoggedNetworkNumber(tunableName + "/kV", initialGains.kV);
        this.tunableKS = new LoggedNetworkNumber(tunableName + "/kS", initialGains.kS);
        this.tunableKA = new LoggedNetworkNumber(tunableName + "/kA", initialGains.kA);
        this.tunableKG = new LoggedNetworkNumber(tunableName + "/kG", initialGains.kG);

        // Initialize cache
        this.lastKP = initialGains.kP;
        this.lastKI = initialGains.kI;
        this.lastKD = initialGains.kD;
        this.lastKV = initialGains.kV;
        this.lastKS = initialGains.kS;
        this.lastKA = initialGains.kA;
        this.lastKG = initialGains.kG;

        // Apply initial configuration
        applyTunableGains();
    }

    public TunableTalonFX(int deviceId, String canbus, String tunableName) {
        this(deviceId, canbus, tunableName, new Slot0Configs());
    }
    /**
     * Call this method periodically (e.g., in your subsystem's periodic()) to check for and apply any PID gain changes
     * from the dashboard.
     *
     * @return true if gains were updated, false otherwise
     */
    public boolean updateTunableGains() {
        // Check if any values changed
        double currentKP = tunableKP.get();
        double currentKI = tunableKI.get();
        double currentKD = tunableKD.get();
        double currentKV = tunableKV.get();
        double currentKS = tunableKS.get();
        double currentKA = tunableKA.get();
        double currentKG = tunableKG.get();

        boolean changed = currentKP != lastKP
                || currentKI != lastKI
                || currentKD != lastKD
                || currentKV != lastKV
                || currentKS != lastKS
                || currentKA != lastKA
                || currentKG != lastKG;

        if (changed) {
            // Update cache
            lastKP = currentKP;
            lastKI = currentKI;
            lastKD = currentKD;
            lastKV = currentKV;
            lastKS = currentKS;
            lastKA = currentKA;
            lastKG = currentKG;

            // Apply new gains
            applyTunableGains();
        }

        return changed;
    }

    /** Applies the current gain values to the motor. */
    private void applyTunableGains() {
        var gains = new Slot0Configs()
                .withKP(lastKP)
                .withKI(lastKI)
                .withKD(lastKD)
                .withKV(lastKV)
                .withKS(lastKS)
                .withKA(lastKA)
                .withKG(lastKG);

        getConfigurator().apply(gains);
    }

    /** Force applies the current dashboard values to the motor. */
    public void forceApplyTunableGains() {
        lastKP = tunableKP.get();
        lastKI = tunableKI.get();
        lastKD = tunableKD.get();
        lastKV = tunableKV.get();
        lastKS = tunableKS.get();
        lastKA = tunableKA.get();
        lastKG = tunableKG.get();
        applyTunableGains();
    }

    /**
     * Gets a Slot0Configs object with the current tunable values.
     *
     * @return A new Slot0Configs with the current dashboard values
     */
    public Slot0Configs getTunableSlot0Configs() {
        return new Slot0Configs()
                .withKP(tunableKP.get())
                .withKI(tunableKI.get())
                .withKD(tunableKD.get())
                .withKV(tunableKV.get())
                .withKS(tunableKS.get())
                .withKA(tunableKA.get())
                .withKG(tunableKG.get());
    }

    // Getters for current tunable values
    public double getTunableKP() {
        return tunableKP.get();
    }

    public double getTunableKI() {
        return tunableKI.get();
    }

    public double getTunableKD() {
        return tunableKD.get();
    }

    public double getTunableKV() {
        return tunableKV.get();
    }

    public double getTunableKS() {
        return tunableKS.get();
    }

    public double getTunableKA() {
        return tunableKA.get();
    }

    public double getTunableKG() {
        return tunableKG.get();
    }

    public String getTunableName() {
        return tunableName;
    }

    /**
     * Applies a TalonFXConfiguration to the motor and updates the tunable NetworkTables values from its Slot0 gains.
     *
     * @param configuration The TalonFXConfiguration to apply
     * @return The StatusCode from applying the configuration
     */
    public StatusCode apply(TalonFXConfiguration configuration) {
        Slot0Configs pidConfigs = configuration.Slot0;

        // Update the tunable NetworkTables values
        tunableKP.set(pidConfigs.kP);
        tunableKI.set(pidConfigs.kI);
        tunableKD.set(pidConfigs.kD);
        tunableKV.set(pidConfigs.kV);
        tunableKS.set(pidConfigs.kS);
        tunableKA.set(pidConfigs.kA);
        tunableKG.set(pidConfigs.kG);

        // Update the cache
        lastKP = pidConfigs.kP;
        lastKI = pidConfigs.kI;
        lastKD = pidConfigs.kD;
        lastKV = pidConfigs.kV;
        lastKS = pidConfigs.kS;
        lastKA = pidConfigs.kA;
        lastKG = pidConfigs.kG;

        return super.getConfigurator().apply(configuration);
    }
}