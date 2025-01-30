package frc.lib.util;

import java.io.Serializable;
import java.util.Objects;
import java.util.RandomAccess;

/**
 * An optimized int array. Used in lieu of an {@code ArrayList<Integer>} to avoid boxing (and the
 * performance implications that come with it).
 */
public class IntArrayList implements RandomAccess, Cloneable, Serializable {
    private static final long serialVersionUID = -7046029254386353130L;

    /**
     * During list creation, many reallocations at low capacities may be common, so we jump from 0
     * to 10 before our normal reallocation scheme.
     */
    public static final int DEFAULT_INITIAL_CAPACITY = 10;

    protected transient int[] backing;
    protected int size;

    /**
     * A static, final, empty array to be used as default array in allocations.
     */
    private static final int[] DEFAULT_EMPTY_ARRAY = {};


    /**
     * Creates a new array list with {@link #DEFAULT_INITIAL_CAPACITY} capacity.
     */
    public IntArrayList() {
        backing = DEFAULT_EMPTY_ARRAY;
    }

    /**
     * Grows this arraylist, ensuring it can contain {@code capacity} longs.
     *
     * @param capacity the new minimum capacity for this array list.
     */
    private void grow(int capacity) {
        if (capacity <= backing.length) {
            return;
        }
        if (backing != DEFAULT_EMPTY_ARRAY) {
            capacity = (int) Math.max(
                Math.min((long) backing.length + (backing.length >> 1), Integer.MAX_VALUE - 8),
                capacity);
        } else if (capacity < DEFAULT_INITIAL_CAPACITY) {
            capacity = DEFAULT_INITIAL_CAPACITY;
        }
        backing = forceCapacity(backing, capacity, size);
        assert size <= backing.length;
    }

    /**
     * Appends {@code k} to the end of this list.
     */
    public void add(final int k) {
        grow(size + 1);
        backing[size++] = k;
        assert size <= backing.length;
    }

    /**
     * Get long element at index {@code index}.
     */
    public int get(int index) {
        Objects.checkIndex(index, size);
        return backing[index];
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
    public int remove(final int index) {
        Objects.checkIndex(index, size);
        final int[] a = this.backing;
        final int old = a[index];
        size--;
        if (index != size) {
            System.arraycopy(a, index + 1, a, index, size);
        }
        assert size <= a.length;
        return old;
    }

    /** Remove all values. */
    public void clear() {
        this.size = 0;
    }

    /**
     * Convert to an array. This performs a copy, since the backing array may be larger than the
     * array needed to be returned.
     */
    public int[] toArray() {
        int[] result = new int[size];
        System.arraycopy(backing, 0, result, 0, size);
        return result;
    }

    private static int[] forceCapacity(final int[] array, final int length, final int preserve) {
        final int[] newArray = new int[length];
        System.arraycopy(array, 0, newArray, 0, preserve);
        return newArray;
    }
}
