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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
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

    private final int deviceId;

    private final boolean TUNING_ENABLED = false;

    // Tunable PID gains
    private final @NotNull LoggedNetworkNumber tunableKP;
    private final @NotNull LoggedNetworkNumber tunableKI;
    private final @NotNull LoggedNetworkNumber tunableKD;
    private final @NotNull LoggedNetworkNumber tunableKV;
    private final @NotNull LoggedNetworkNumber tunableKS;
    private final @NotNull LoggedNetworkNumber tunableKA;
    private final @NotNull LoggedNetworkNumber tunableKG;

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
    public TunableTalonFX(int deviceId, String canbus, String tunableName, @NotNull Slot0Configs initialGains) {
        super(deviceId, new CANBus(canbus));
        this.tunableName = tunableName;
        this.deviceId = deviceId;

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
        this.applyTunableGains();
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

        String motorName = "Temp/" + this.tunableName + "/" + this.deviceId;
        Logger.recordOutput(motorName, this.getDeviceTemp().getValue().in(Units.Fahrenheit));

        if (Constants.motorTempWarningThreshold < this.getDeviceTemp().getValue().in(Units.Fahrenheit)) {
            DriverStation.reportWarning(
                    "MOTOR OVERHEATING: " + motorName, Thread.currentThread().getStackTrace());
        }

        if (!this.TUNING_ENABLED) return false;
        // Check if any values changed
        double currentKP = this.tunableKP.get();
        double currentKI = this.tunableKI.get();
        double currentKD = this.tunableKD.get();
        double currentKV = this.tunableKV.get();
        double currentKS = this.tunableKS.get();
        double currentKA = this.tunableKA.get();
        double currentKG = this.tunableKG.get();

        boolean changed = currentKP != this.lastKP
                || currentKI != this.lastKI
                || currentKD != this.lastKD
                || currentKV != this.lastKV
                || currentKS != this.lastKS
                || currentKA != this.lastKA
                || currentKG != this.lastKG;

        if (changed) {
            // Update cache
            this.lastKP = currentKP;
            this.lastKI = currentKI;
            this.lastKD = currentKD;
            this.lastKV = currentKV;
            this.lastKS = currentKS;
            this.lastKA = currentKA;
            this.lastKG = currentKG;

            // Apply new gains
            this.applyTunableGains();
        }

        return changed;
    }

    /** Applies the current gain values to the motor. */
    private void applyTunableGains() {
        var gains = new Slot0Configs()
                .withKP(this.lastKP)
                .withKI(this.lastKI)
                .withKD(this.lastKD)
                .withKV(this.lastKV)
                .withKS(this.lastKS)
                .withKA(this.lastKA)
                .withKG(this.lastKG);

        this.getConfigurator().apply(gains);
    }

    /** Force applies the current dashboard values to the motor. */
    public void forceApplyTunableGains() {
        this.lastKP = this.tunableKP.get();
        this.lastKI = this.tunableKI.get();
        this.lastKD = this.tunableKD.get();
        this.lastKV = this.tunableKV.get();
        this.lastKS = this.tunableKS.get();
        this.lastKA = this.tunableKA.get();
        this.lastKG = this.tunableKG.get();
        this.applyTunableGains();
    }

    /**
     * Gets a Slot0Configs object with the current tunable values.
     *
     * @return A new Slot0Configs with the current dashboard values
     */
    public @NotNull Slot0Configs getTunableSlot0Configs() {
        return new Slot0Configs()
                .withKP(this.tunableKP.get())
                .withKI(this.tunableKI.get())
                .withKD(this.tunableKD.get())
                .withKV(this.tunableKV.get())
                .withKS(this.tunableKS.get())
                .withKA(this.tunableKA.get())
                .withKG(this.tunableKG.get());
    }

    // Getters for current tunable values
    public double getTunableKP() {
        return this.tunableKP.get();
    }

    public double getTunableKI() {
        return this.TUNING_ENABLED ? this.tunableKI.get() : 0.0;
    }

    public double getTunableKD() {
        return this.TUNING_ENABLED ? this.tunableKD.get() : 0.0;
    }

    public double getTunableKV() {
        return this.TUNING_ENABLED ? this.tunableKV.get() : 0.0;
    }

    public double getTunableKS() {
        return this.TUNING_ENABLED ? this.tunableKS.get() : 0.0;
    }

    public double getTunableKA() {
        return this.TUNING_ENABLED ? this.tunableKA.get() : 0.0;
    }

    public double getTunableKG() {
        return this.TUNING_ENABLED ? this.tunableKG.get() : 0.0;
    }

    public String getTunableName() {
        return this.tunableName;
    }

    /**
     * Applies a TalonFXConfiguration to the motor and updates the tunable NetworkTables values from its Slot0 gains.
     *
     * @param configuration The TalonFXConfiguration to apply
     * @return The StatusCode from applying the configuration
     */
    public StatusCode applyConfiguration(@NotNull TalonFXConfiguration configuration) {
        Slot0Configs pidConfigs = configuration.Slot0;

        // Update the tunable NetworkTables values
        this.tunableKP.set(pidConfigs.kP);
        this.tunableKI.set(pidConfigs.kI);
        this.tunableKD.set(pidConfigs.kD);
        this.tunableKV.set(pidConfigs.kV);
        this.tunableKS.set(pidConfigs.kS);
        this.tunableKA.set(pidConfigs.kA);
        this.tunableKG.set(pidConfigs.kG);

        // Update the cache
        this.lastKP = pidConfigs.kP;
        this.lastKI = pidConfigs.kI;
        this.lastKD = pidConfigs.kD;
        this.lastKV = pidConfigs.kV;
        this.lastKS = pidConfigs.kS;
        this.lastKA = pidConfigs.kA;
        this.lastKG = pidConfigs.kG;

        return super.getConfigurator().apply(configuration);
    }

    public StatusCode applyConfiguration(@NotNull TalonFXConfiguration configuration, double timeoutSeconds) {
        Slot0Configs pidConfigs = configuration.Slot0;

        // Update the tunable NetworkTables values
        this.tunableKP.set(pidConfigs.kP);
        this.tunableKI.set(pidConfigs.kI);
        this.tunableKD.set(pidConfigs.kD);
        this.tunableKV.set(pidConfigs.kV);
        this.tunableKS.set(pidConfigs.kS);
        this.tunableKA.set(pidConfigs.kA);
        this.tunableKG.set(pidConfigs.kG);

        // Update the cache
        this.lastKP = pidConfigs.kP;
        this.lastKI = pidConfigs.kI;
        this.lastKD = pidConfigs.kD;
        this.lastKV = pidConfigs.kV;
        this.lastKS = pidConfigs.kS;
        this.lastKA = pidConfigs.kA;
        this.lastKG = pidConfigs.kG;

        return super.getConfigurator().apply(configuration, timeoutSeconds);
    }
}
