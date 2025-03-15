package frc.robot;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AlertManager {
    private static final double MINIMUM_DISPLAY_TIME = 0.1;
    private static final Set<Command> commands = new HashSet<>();
    private static final Map<String, AlertTracker> alertTrackers = new HashMap<>();

    public static void initialize() {
        CommandScheduler.getInstance().onCommandExecute((Command command) -> {
            handleExecute(command);
        });

        CommandScheduler.getInstance().onCommandFinish((Command command) -> {
            handleEnd(command);
        });

        CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
            handleEnd(command);
        });
    }

    private static void handleExecute(Command command) {
        if (!commands.contains(command)) {
            commands.add(command);
            
            String name = command.getName();
            AlertTracker tracker = alertTrackers.computeIfAbsent(
                name,
                (String k) -> new AlertTracker(name, MINIMUM_DISPLAY_TIME)
            );
            tracker.increment();
        }
    }

    private static void handleEnd(Command command) {
        if (commands.remove(command)) {
            String name = command.getName();
            AlertTracker tracker = alertTrackers.get(name);
            if (tracker != null) {
                tracker.decrement();
                if (tracker.getCount() == 0) {
                    alertTrackers.remove(name);
                }
            }
        }
    }

    private static class AlertTracker {
        private final Alert alert;
        private final Notifier hide;
        private int count;
        private final double MIN_DISPLAY_TIME;

        public AlertTracker(String commandName, double minTime) {
            alert = new Alert(commandName, AlertType.kInfo);
            hide = new Notifier(() -> alert.set(false));
            MIN_DISPLAY_TIME = minTime;
            count = 0;
        }

        public synchronized void increment() {
            count++;
            if (count == 1) {
                alert.set(true);
                hide.stop();
            }
        }

        public synchronized void decrement() {
            count--;
            if (count == 0) {
                hide.startSingle(MIN_DISPLAY_TIME);
            }
        }

        public int getCount() {
            return count;
        }
    }
}
