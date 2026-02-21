package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestNavConstants {
    public static final Transform3d ROBOT_TO_QUEST =
            new Transform3d(0.0, 0.0, 0.0, new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90)));

            public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
}
