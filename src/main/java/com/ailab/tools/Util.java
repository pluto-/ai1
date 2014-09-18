package com.ailab.tools;

import java.awt.Point;

public class Util{

    /**
     * Returns closest point on segment to point
     *
     * @param ss
     *            segment start point
     * @param se
     *            segment end point
     * @param p
     *            point to found closest point on segment
     * @return closest point on segment to p
     */
    public static Position getClosestPointOnSegment(Position ss, Position se, Position p)
    {
        return getClosestPointOnSegment(ss.getX(), ss.getY(), se.getX(), se.getY(), p.getX(), p.getY());
    }

    /**
     * Returns closest point on segment to point
     *
     * @param sx1
     *            segment x coord 1
     * @param sy1
     *            segment y coord 1
     * @param sx2
     *            segment x coord 2
     * @param sy2
     *            segment y coord 2
     * @param px
     *            point x coord
     * @param py
     *            point y coord
     * @return closets point on segment to point
     */
    public static Position getClosestPointOnSegment(double sx1, double sy1, double sx2, double sy2, double px, double py)
    {
        double xDelta = sx2 - sx1;
        double yDelta = sy2 - sy1;

        if ((xDelta == 0.0) && (yDelta == 0.0))
        {
            throw new IllegalArgumentException("Segment start equals segment end");
        }

        double u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

        final Position closestPoint;
        if (u < 0.0)
        {
            closestPoint = new Position(sx1, sy1);
        }
        else if (u > 1.0)
        {
            closestPoint = new Position(sx2, sy2);
        }
        else
        {
            closestPoint = new Position(sx1 + u * xDelta, sy1 + u * yDelta);
        }

        return closestPoint;
    }
}