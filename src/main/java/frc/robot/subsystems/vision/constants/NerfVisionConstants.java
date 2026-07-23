package frc.robot.subsystems.vision.constants;

public class NerfVisionConstants {
    public static final VisionConstants DATA;

    static {
        VisionConstants base = BaseVisionConstants.DATA;

        DATA = new VisionConstants(
                base.aprilTagLayout(),
                base.camera0Name(),
                base.camera1Name(),
                base.robotToCamera0(),
                base.robotToCamera1(),
                base.robotToQuest(),
                base.maxAmbiguity(),
                base.maxZError(),
                base.linearStdDevBaseline(),
                base.angularStdDevBaseline(),
                base.questNavStdDevs(),
                base.cameraStdDevFactors(),
                base.linearStdDevMegatag2Factor(),
                base.angularStdDevMegatag2Factor(),
                1);
    }
}
