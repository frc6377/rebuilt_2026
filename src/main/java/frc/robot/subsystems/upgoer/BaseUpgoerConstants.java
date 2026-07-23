package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

public class BaseUpgoerConstants {
    public static final UpgoerConstants DATA = new UpgoerConstants(
            "rio",
            1.0, // defaultKP
            0.0, // defaultKI
            0.0, // defaultKD
            0.0, // defaultKV
            0.0, // defaultKS
            Amps.of(50.0),
            Amps.of(35.0),
            RPM.of(4500.0),
            RPM.of(-4500.0));
}
