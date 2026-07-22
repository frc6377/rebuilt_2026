// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj.Threads;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Latest-value-wins asynchronous applier for TalonFX {@link CurrentLimitsConfigs}.
 *
 * <p>Mirrors Mechanical Advantage's TalonFXCurrentConfigurator: the robot thread only queues a request; a background
 * worker performs Phoenix {@code apply()} so CAN config traffic cannot stall the main loop. Failed applies retry after
 * a short delay unless a newer request arrives.
 */
public class TalonFXCurrentConfigurator {
    private final TalonFXConfigurator configurator;
    private final ReentrantLock mutex = new ReentrantLock();
    private final Condition cv = mutex.newCondition();
    private final CurrentLimitsConfigs targetConfig = new CurrentLimitsConfigs();

    private boolean hasInitial = false;
    private boolean needsApply = false;
    private boolean isRetry = false;
    private volatile boolean running = true;

    public TalonFXCurrentConfigurator(TalonFXConfigurator configurator) {
        this.configurator = configurator;
        Thread worker = new Thread(this::workerLoop, "TalonFXCurrentConfigurator");
        worker.setDaemon(true);
        worker.start();
    }

    /** Queue a current-limits config. Identical configs are ignored; newer requests supersede. */
    public void setConfig(CurrentLimitsConfigs config) {
        mutex.lock();
        try {
            if (hasInitial
                    && targetConfig.StatorCurrentLimit == config.StatorCurrentLimit
                    && targetConfig.StatorCurrentLimitEnable == config.StatorCurrentLimitEnable
                    && targetConfig.SupplyCurrentLimit == config.SupplyCurrentLimit
                    && targetConfig.SupplyCurrentLimitEnable == config.SupplyCurrentLimitEnable
                    && targetConfig.SupplyCurrentLowerLimit == config.SupplyCurrentLowerLimit
                    && targetConfig.SupplyCurrentLowerTime == config.SupplyCurrentLowerTime) {
                return;
            }
            hasInitial = true;
            copy(targetConfig, config);
            needsApply = true;
            isRetry = false;
            cv.signalAll();
        } finally {
            mutex.unlock();
        }
    }

    private void workerLoop() {
        Threads.setCurrentThreadPriority(false, 0);
        mutex.lock();
        try {
            while (running) {
                while (running && !needsApply) {
                    cv.await();
                }
                if (!running) {
                    break;
                }

                if (isRetry) {
                    cv.await(20, TimeUnit.MILLISECONDS);
                    if (!running) {
                        break;
                    }
                }

                CurrentLimitsConfigs configToApply = new CurrentLimitsConfigs();
                copy(configToApply, targetConfig);
                needsApply = false;
                isRetry = false;

                mutex.unlock();
                StatusCode status;
                try {
                    status = configurator.apply(configToApply);
                } finally {
                    mutex.lock();
                }

                if (!status.isOK() && !needsApply) {
                    needsApply = true;
                    isRetry = true;
                }
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            mutex.unlock();
        }
    }

    private static void copy(CurrentLimitsConfigs to, CurrentLimitsConfigs from) {
        to.StatorCurrentLimit = from.StatorCurrentLimit;
        to.StatorCurrentLimitEnable = from.StatorCurrentLimitEnable;
        to.SupplyCurrentLimit = from.SupplyCurrentLimit;
        to.SupplyCurrentLimitEnable = from.SupplyCurrentLimitEnable;
        to.SupplyCurrentLowerLimit = from.SupplyCurrentLowerLimit;
        to.SupplyCurrentLowerTime = from.SupplyCurrentLowerTime;
    }
}
