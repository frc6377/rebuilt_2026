package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public record UpgoerConstants(
        String canBusName,
        double defaultKP,
        double defaultKI,
        double defaultKD,
        double defaultKV,
        double defaultKS,
        Current currentLimit,
        Current supplyCurrentLimit,
        AngularVelocity defaultFeedVelocity,
        AngularVelocity defaultUnjamVelocity) {}
