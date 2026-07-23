package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

public class NerfUpgoerConstants {
    public static final UpgoerConstants DATA = new UpgoerConstants(
            BaseUpgoerConstants.DATA.canBusName(),
            BaseUpgoerConstants.DATA.defaultKP(),
            BaseUpgoerConstants.DATA.defaultKI(),
            BaseUpgoerConstants.DATA.defaultKD(),
            BaseUpgoerConstants.DATA.defaultKV(),
            BaseUpgoerConstants.DATA.defaultKS(),
            BaseUpgoerConstants.DATA.currentLimit(),
            BaseUpgoerConstants.DATA.supplyCurrentLimit(),
            RPM.of(2000.0), // defaultFeedVelocity (nerfed from 4500)
            RPM.of(-2000.0) // defaultUnjamVelocity (nerfed from -4500)
            );
}
