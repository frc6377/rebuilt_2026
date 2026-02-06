// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    /** Creates a new Indexer. */
    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    /** Run the indexer at the specified speed (-1.0 to 1.0). */
    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

    /** Stop the indexer. */
    public void stop() {
        io.stop();
    }

    // ==================== Command Factory Methods ====================

    /**
     * Creates a command to run the indexer at a specified speed. Stops when command ends.
     *
     * @param speed The duty cycle speed (-1.0 to 1.0).
     * @return A command that runs the indexer.
     */
    public Command runCommand(double speed) {
        return runEnd(() -> setSpeed(speed), this::stop).withName("Indexer.Run");
    }

    /**
     * Creates a command to run the indexer at a dynamically supplied speed. Stops when command ends.
     *
     * @param speedSupplier A supplier for the speed.
     * @return A command that runs the indexer.
     */
    public Command runCommand(DoubleSupplier speedSupplier) {
        return runEnd(() -> setSpeed(speedSupplier.getAsDouble()), this::stop).withName("Indexer.RunDynamic");
    }

    /** Creates a command to feed game pieces forward at the default feed speed. */
    public Command feedCommand() {
        return runCommand(IndexerConstants.kFeedSpeed).withName("Indexer.Feed");
    }

    /** Creates a command to reverse/eject game pieces at the default reverse speed. */
    public Command reverseCommand() {
        return runCommand(IndexerConstants.kReverseSpeed).withName("Indexer.Reverse");
    }

    /** Creates a command to stop the indexer. */
    public Command stopCommand() {
        return runOnce(this::stop).withName("Indexer.Stop");
    }
}
