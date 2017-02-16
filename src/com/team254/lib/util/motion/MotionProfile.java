package com.team254.lib.util.motion;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import com.team254.lib.util.motion.MotionState;

public class MotionProfile {
    protected List<MotionSegment> mSegments;

    public MotionProfile() {
        mSegments = new ArrayList<>();
    }

    public MotionProfile(List<MotionSegment> segments) {
        mSegments = segments;
    }

    /**
     * Checks if the given MotionProfile is valid. This checks that:
     * 
     * 1. All segments are valid.
     * 
     * 2. All segments are C1 continuous in position and C0 continuous in velocity.
     * 
     * @return True if the MotionProfile is valid.
     */
    public boolean isValid() {
        MotionSegment prev_segment = null;
        for (MotionSegment s : mSegments) {
            if (!s.isValid()) {
                return false;
            }
            if (prev_segment != null && !s.start().coincident(prev_segment.end())) {
                // Adjacent segments are not continuous.
                System.err.println("Segments not continuous! End: " + prev_segment.end() + ", Start: " + s.start());
                return false;
            }
            prev_segment = s;
        }
        return true;
    }

    public boolean isEmpty() {
        return mSegments.isEmpty();
    }

    public Optional<MotionState> stateByTime(double t) {
        for (MotionSegment s : mSegments) {
            if (s.start().t() <= t && s.end().t() >= t) {
                return Optional.of(s.start().extrapolate(t));
            }
        }
        return Optional.empty();
    }

    public Optional<MotionState> firstStateByPos(double pos) {
        for (MotionSegment s : mSegments) {
            final boolean vel_positive = s.start().vel() > 0.0 || s.end().vel() > 0.0;
            if (vel_positive ? (s.start().pos() <= pos && s.end().pos() >= pos)
                    : (s.start().pos() >= pos && s.end().pos() <= pos)) {
                final double t = s.start().nextTimeAtPos(pos);
                if (Double.isNaN(t)) {
                    System.err.println("Error! We should reach 'pos' but we don't");
                    return Optional.empty();
                }
                return Optional.of(s.start().extrapolate(t));
            }
        }
        // We never reach pos.
        return Optional.empty();
    }

    public void trimBeforeTime(double t) {
        for (Iterator<MotionSegment> iterator = mSegments.iterator(); iterator.hasNext();) {
            MotionSegment s = iterator.next();
            if (s.end().t() <= t) {
                // Segment is fully before t.
                iterator.remove();
                continue;
            }
            if (s.start().t() <= t) {
                // Segment begins before t; let's shorten the segment.
                s.setStart(s.start().extrapolate(t));
            }
            break;
        }
    }

    public void clear() {
        mSegments.clear();
    }

    public void reset(MotionState initial_state) {
        clear();
        mSegments.add(new MotionSegment(initial_state, initial_state));
    }

    /**
     * Remove redundant segments (segments whose start and end states are coincident).
     */
    public void consolidate() {
        for (Iterator<MotionSegment> iterator = mSegments.iterator(); iterator.hasNext() && mSegments.size() > 1;) {
            MotionSegment s = iterator.next();
            if (s.start().coincident(s.end())) {
                iterator.remove();
            }
        }
    }

    public void appendControl(double acc, double dt) {
        if (isEmpty()) {
            System.err.println("Error!  Trying to append to empty profile");
            return;
        }
        MotionState last_end_state = mSegments.get(mSegments.size() - 1).end();
        MotionState new_start_state = new MotionState(last_end_state.t(), last_end_state.pos(), last_end_state.vel(),
                acc);
        mSegments.add(new MotionSegment(new_start_state, new_start_state.extrapolate(new_start_state.t() + dt)));
    }

    public void appendSegment(MotionSegment segment) {
        mSegments.add(segment);
    }

    public void appendProfile(MotionProfile profile) {
        for (MotionSegment s : profile.segments()) {
            appendSegment(s);
        }
    }

    public int size() {
        return mSegments.size();
    }

    public List<MotionSegment> segments() {
        return mSegments;
    }

    public MotionState startState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return mSegments.get(0).start();
    }

    public double startTime() {
        if (isEmpty()) {
            return Double.NaN;
        }
        return mSegments.get(0).start().t();
    }

    public double startPos() {
        if (isEmpty()) {
            return Double.NaN;
        }
        return mSegments.get(0).start().pos();
    }

    public MotionState endState() {
        if (isEmpty()) {
            return MotionState.kInvalidState;
        }
        return mSegments.get(mSegments.size() - 1).end();
    }

    public double endTime() {
        if (isEmpty()) {
            return Double.NaN;
        }
        return mSegments.get(mSegments.size() - 1).end().t();
    }

    public double endPos() {
        if (isEmpty()) {
            return Double.NaN;
        }
        return mSegments.get(mSegments.size() - 1).end().pos();
    }

    public double duration() {
        return endTime() - startTime();
    }

    public double length() {
        double length = 0.0;
        for (MotionSegment s : segments()) {
            length += Math.abs(s.end().pos() - s.start().pos());
        }
        return length;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("Profile:");
        for (MotionSegment s : segments()) {
            builder.append("\n\t");
            builder.append(s);
        }
        return builder.toString();
    }
}