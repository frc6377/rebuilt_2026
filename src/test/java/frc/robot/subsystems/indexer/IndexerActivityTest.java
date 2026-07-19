package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class IndexerActivityTest {
    @Test
    void percentOutputCommandsUpdateActivity() {
        var indexer = new Indexer(new IndexerIO() {});

        indexer.setCustomSpeed(0.4).initialize();
        assertTrue(indexer.isPowerManagementActive());

        indexer.runPercentOutput(-0.4).execute();
        assertTrue(indexer.isPowerManagementActive());

        indexer.runPercentOutput(0.0).execute();
        assertFalse(indexer.isPowerManagementActive());
    }

    @Test
    void velocitySupplierUpdatesActivityEveryExecution() {
        var enabled = new AtomicBoolean(true);
        var indexer = new Indexer(new IndexerIO() {});
        var command = indexer.index(enabled::get);

        command.execute();
        assertTrue(indexer.isPowerManagementActive());

        enabled.set(false);
        command.execute();
        assertFalse(indexer.isPowerManagementActive());
    }

    @Test
    void indexAndReverseCommandsUpdateActivity() {
        var indexer = new Indexer(new IndexerIO() {});

        var indexCommand = indexer.index();
        indexCommand.execute();
        assertTrue(indexer.isPowerManagementActive());
        indexCommand.end(false);
        assertFalse(indexer.isPowerManagementActive());

        indexer.indexReverse().execute();
        assertTrue(indexer.isPowerManagementActive());
    }

    @Test
    void directRunningAndStopPathsUpdateActivity() {
        var indexer = new Indexer(new IndexerIO() {});

        indexer.setRunning(true);
        assertTrue(indexer.isPowerManagementActive());

        indexer.setRunning(false);
        assertFalse(indexer.isPowerManagementActive());

        indexer.setRunning(true);
        indexer.stop().initialize();
        assertFalse(indexer.isPowerManagementActive());
    }

    @Test
    void continuousCommandsClearActivityWhenTheyEnd() {
        var indexer = new Indexer(new IndexerIO() {});

        var percentCommand = indexer.runPercentOutput(0.4);
        percentCommand.execute();
        percentCommand.end(true);
        assertFalse(indexer.isPowerManagementActive());

        var reverseCommand = indexer.indexReverse();
        reverseCommand.execute();
        reverseCommand.end(true);
        assertFalse(indexer.isPowerManagementActive());

        var supplierCommand = indexer.index(() -> true);
        supplierCommand.execute();
        supplierCommand.end(true);
        assertFalse(indexer.isPowerManagementActive());
    }
}
