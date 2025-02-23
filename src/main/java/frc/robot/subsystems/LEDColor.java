package frc.robot.subsystems;

public enum LEDColor {
    ORANGE, YELLOW, RED, GREEN, BLUE, VIOLET;

    @Override
    public String toString() {
        return this.name();
    }
}
