package frc.robot.utils;

public class LatchedValue<T> {
    private T value;
    private boolean isSet;

    public LatchedValue(T defaultValue) {
        value = defaultValue;
        isSet = false;
    }

    public T getValue() {
        return value;
    }

    public boolean setValue(T valToSet) {
        if (!isSet) {
            value = valToSet;
            isSet = true;
            return true;
        } else {
            return false;
        }
    }

    public boolean valueIsSet() {
        return isSet;
    }
}
