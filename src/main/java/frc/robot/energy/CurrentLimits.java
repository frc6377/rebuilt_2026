// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

public final class CurrentLimits {
    public static final class Shared {
        public static final double stepAmps = 0.5;
        public static final double recoveryRateAmpsPerSec = 50.0;
    }

    public static final class Static {
        public static final double driveSupplyAmps = 50;
        public static final double rollerSupplyAmps = 50;
        public static final double extenderSupplyAmps = 25;
        public static final double indexerSupplyAmps = 25;
    }

    public static final class Drive {
        public static final int motorCount = 4;
        public static final double minLimitAmps = 5;
        public static final double maxLimitAmps = 35;
        public static final double targetAmps = maxLimitAmps * motorCount;
        public static final double autoLimitAmps = 50;
        public static final double standbyAmps = minLimitAmps * motorCount;
    }

    public static final class Indexer {
        public static final double standbyAmps = 5;
        public static final double activeMinAmps = 15;
        public static final double targetAmps = 25;
        public static final double maxAmps = 25;
    }

    public static final class Extender {
        public static final double standbyAmps = 5;
        public static final double activeMinAmps = 12;
        public static final double targetAmps = 25;
        public static final double maxAmps = 25;
    }

    public static final class Roller {
        public static final int motorCount = 2;
        public static final double offMinAmps = 5;
        public static final double idleMinAmps = 10;
        public static final double activeMinAmps = 20;
        public static final double targetAmps = 40;
        public static final double maxAmps = 100;
    }
}
