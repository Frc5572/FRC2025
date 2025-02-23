package frc.lib.util;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Objects;
import java.util.function.IntFunction;

/** An array-backed list with fixed capacity. */
public final class SmallVec<T> {

    private final Object[] backing;
    private int size = 0;

    /** Create a new SmallVec with given capacity */
    public SmallVec(int capacity) {
        backing = new Object[capacity];
    }

    /**
     * Appends {@code value} to the end of this list.
     */
    public void add(final T value) {
        backing[size++] = value;
    }

    /**
     * Get long element at index {@code index}.
     */
    @SuppressWarnings("unchecked")
    public T get(int index) {
        Objects.checkIndex(index, size);
        return (T) backing[index];
    }

    /**
     * Checks if the list has no elements.
     */
    public boolean isEmpty() {
        return size == 0;
    }

    /**
     * Returns the number of elements in this list.
     */
    public int size() {
        return size;
    }

    /**
     * Removes the element at the specified position in this list.
     */
    @SuppressWarnings("unchecked")
    public T remove(final int index) {
        Objects.checkIndex(index, size);
        final Object[] a = this.backing;
        final Object old = a[index];
        size--;
        if (index != size) {
            System.arraycopy(a, index + 1, a, index, size);
        }
        assert size <= a.length;
        return (T) old;
    }

    /** Remove all values. */
    public void clear() {
        this.size = 0;
    }

    /**
     * Convert to an array. This performs a copy, since the backing array may be larger than the
     * array needed to be returned.
     */
    public T[] toArray(IntFunction<T[]> newFunc) {
        T[] result = newFunc.apply(size);
        System.arraycopy(backing, 0, result, 0, size);
        return result;
    }

    /** Sort the list */
    @SuppressWarnings("unchecked")
    public void sort(Comparator<? super T> c) {
        Arrays.sort((T[]) backing, 0, size, c);
    }

}
