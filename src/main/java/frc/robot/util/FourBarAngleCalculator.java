package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class FourBarAngleCalculator {

    public record Geometry(
            Translation3d outputPivot,
            Translation3d inputPivot,
            Translation3d couplerAtReference,
            Distance outputLinkLength,
            Distance inputLinkLength,
            Angle outputReferenceAngle,
            Angle inputReferenceAngle) {}

    private final Geometry geometry;
    private final Transform3d outputPivotToJoint;
    private final Transform3d inputPivotToJoint;
    private final Translation3d couplerOffsetFromInputJoint;

    public FourBarAngleCalculator(Geometry geometry) {
        this.geometry = geometry;

        Translation3d outputJointAtReference = jointFromReferenceAngle(
                geometry.outputPivot(), geometry.outputLinkLength().in(Meters), geometry.outputReferenceAngle());
        Translation3d inputJointAtReference = jointFromReferenceAngle(
                geometry.inputPivot(), geometry.inputLinkLength().in(Meters), geometry.inputReferenceAngle());

        this.outputPivotToJoint =
                new Transform3d(outputJointAtReference.minus(geometry.outputPivot()), new Rotation3d());
        this.inputPivotToJoint = new Transform3d(inputJointAtReference.minus(geometry.inputPivot()), new Rotation3d());
        this.couplerOffsetFromInputJoint = geometry.couplerAtReference().minus(inputJointAtReference);
    }

    public Pose3d getInputLinkPose(Angle inputAngle) {
        return new Pose3d(geometry.inputPivot(), new Rotation3d(0.0, inputAngle.in(Radians), 0.0));
    }

    public Pose3d getOutputLinkPose(Angle inputAngle) {
        return new Pose3d(
                geometry.outputPivot(),
                new Rotation3d(0.0, getOutputAngle(inputAngle).in(Radians), 0.0));
    }

    public Pose3d getCouplerPose(Angle inputAngle) {
        return new Pose3d(getInputJoint(inputAngle).plus(couplerOffsetFromInputJoint), new Rotation3d());
    }

    public Pose3d getInputJointPose(Angle inputAngle) {
        return new Pose3d(getInputJoint(inputAngle), new Rotation3d());
    }

    public Pose3d getOutputJointPose(Angle inputAngle) {
        return new Pose3d(getOutputLinkPose(inputAngle).plus(outputPivotToJoint).getTranslation(), new Rotation3d());
    }

    public Angle getOutputAngle(Angle inputAngle) {
        return inputAngle;
    }

    private Translation3d getInputJoint(Angle inputAngle) {
        return getInputLinkPose(inputAngle).plus(inputPivotToJoint).getTranslation();
    }

    private static Translation3d jointFromReferenceAngle(
            Translation3d pivot, double lengthMeters, Angle referenceAngle) {
        double angleRad = referenceAngle.in(Radians);
        return new Translation3d(
                pivot.getX() + lengthMeters * Math.cos(angleRad),
                pivot.getY(),
                pivot.getZ() + lengthMeters * Math.sin(angleRad));
    }
}
