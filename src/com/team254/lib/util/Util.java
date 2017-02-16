package com.team254.lib.util;

import java.util.List;

import org.hamcrest.BaseMatcher;
import org.hamcrest.Description;
import org.hamcrest.Matcher;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    
    public static Matcher<Double> epsilonEqualTo(final double b, final double epsilon) {
        return new BaseMatcher<Double>() {

            @Override
            public boolean matches(Object a) {
                return epsilonEquals((Double)a, b, epsilon);
            }

            @Override
            public void describeTo(Description description) {
                description.appendText("Should be within +/- ").appendValue(epsilon).appendText(" of ").appendValue(b);
            }
        };
    }
}
