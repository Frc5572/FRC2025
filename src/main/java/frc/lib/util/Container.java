package frc.lib.util;

/** Stores mutable a value */
public class Container<T> {

    /** The value being stored */
    public T value;

    /** Create new container with initial value. */
    public Container(T value) {
        this.value = value;
    }

}
