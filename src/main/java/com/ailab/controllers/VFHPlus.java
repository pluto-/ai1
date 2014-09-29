package com.ailab.controllers;

import java.util.*;

/**
 * This class handles calculations of the VFH+ algorithm. It contains a number of constants and thresholds which are
 * used in the calculations.
 *
 * The method to use for calculating the steering angle from the VFH+ algorithm is calculateSteeringDirection.
 *
 * Created by Jonas on 2014-09-22.
 */
public class VFHPlus {

    public static final double A = 1.0;
    public static final double ROBOT_RADIUS = 0.25;
    public static final double D_S = 0.05;
    public static final double ENLARGEMENT_RADIUS = ROBOT_RADIUS + D_S;
    public static final int NO_OF_SECTORS = 90;
    public static final int READS_PER_SECTOR = 270 / NO_OF_SECTORS;
    public static final int NO_OF_BLIND_SECTORS = (int)(NO_OF_SECTORS / 3);
    public static final double THRESHOLD_LOW = 0.2;
    public static final double THRESHOLD_HIGH = 0.5;
    public static final int S_MAX = 5;

    public static final double MU1 = 0.8;
    public static final double MU2 = 0.4;
    public static final double MU3 = 0.2;

    private double startAngle;
    private double endAngle;
    private double angleIncrement;
    private double angularResolution;
    private Integer previousTargetSector;

    /**
     * Constructor.
     *
     * @param startAngle the start angle of the laser sensor.
     * @param endAngle the end angle of the laser sensor.
     * @param angleIncrement the angle increment between the echoes of the laser sensor.
     */
    public VFHPlus (double startAngle, double endAngle, double angleIncrement) {
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.angleIncrement = angleIncrement;
        angularResolution = READS_PER_SECTOR * angleIncrement;

        previousTargetSector = null;
    }


    public void resetPreviousTargetSector() {
        previousTargetSector = null;
    }

    /**
     * This method executes the VFH+ algorithm and returns the calculated steering angle.
     * @param distanceToCarrotPoint distance to carrot point.
     * @param distances the distances to the obstacles.
     * @param goalAngle the goal angle calculated by "Follow the Carrot".
     * @return the given goal angle if no dangerous obstacles are found, otherwise the new goal angle calculated by the
     * VFH+ algorithm.
     */
    public Double calculateSteeringDirection(double distanceToCarrotPoint, double distances[], double goalAngle) {
        Map<Integer, Integer> binaryHistogram = buildBinaryHistogram(buildPolarHistogram(distanceToCarrotPoint, distances));
        if (binaryHistogram == null) {
            return goalAngle;
        }

        return findBestDirection(binaryHistogram, goalAngle);
    }

    /**
     * Checks the cost of each candidate direction and chooses the one with lowest cost.
     *
     * @param binaryHistogram the binary histogram.
     * @param goalAngle the goal angle calculated by "Follow the Carrot".
     * @return the direction of the candidate direction with lowest cost. Returns null if no best direction is found.
     */
    private Double findBestDirection(Map<Integer, Integer> binaryHistogram, double goalAngle) {
        int goalSector;
        if(goalAngle < startAngle) {
            goalAngle = startAngle;
        } else if(goalAngle > endAngle) {
            goalAngle = endAngle;
        }
        goalSector = (int)((goalAngle) / angularResolution);

        ArrayList<CandidateValley> candidateValleys = getCandidateValleys(binaryHistogram);
        ArrayList<Integer> candidateDirections = getCandidateDirections(candidateValleys, goalSector);

        double bestCost = Double.MAX_VALUE;
        Integer bestDirection = null;
        for (int i = 0; i < candidateDirections.size(); i++) {
            double candidateCost = calculateCost(candidateDirections.get(i), goalSector, MU1, MU2, MU3);
            if(candidateCost < bestCost) {
                bestCost = candidateCost;
                bestDirection = candidateDirections.get(i);
            }
        }
        previousTargetSector = bestDirection;

        return bestDirection == null ? null : (bestDirection) * angularResolution;
    }


    /**
     * Calculates the cost of the given sector.
     *
     * @param candidateSector the sector which will have it's cost return.
     * @param goalSector the sector number which cintains the goal.
     * @param mu1 First constant.
     * @param mu2 Second Constant.
     * @param mu3 Third Constant.
     * @return the cost of the sector.
     */
    private double calculateCost(int candidateSector, int goalSector, double mu1, double mu2, double mu3) {

        double first = mu1*(calcDeltaSectors(candidateSector, goalSector));
        double second = mu2 * calcDeltaSectors(candidateSector, 0);
        double third = (previousTargetSector == null ? 0 : mu3 * calcDeltaSectors(candidateSector, previousTargetSector));

        return first + second + third;
    }

    /**
     * Calculates how many sectors are between the two given sectors.
     *
     * @param sectorOne the first sector.
     * @param sectorTwo the second sector.
     * @return number of sectors between the given sectors.
     */
    private static int calcDeltaSectors(int sectorOne, int sectorTwo) {
        int first = Math.abs(sectorTwo - sectorOne);
        int second = Math.abs(sectorTwo - sectorOne - (NO_OF_SECTORS + NO_OF_BLIND_SECTORS));
        int third = Math.abs(sectorTwo - sectorOne + (NO_OF_SECTORS + NO_OF_BLIND_SECTORS));
        int min_first_second = Math.min(first, second);
        return Math.min(min_first_second, third);
    }

    /**
     * Creates a list of candidate directions (in sector numbers) from the given candidate valleys. Wide valleys gives
     * two candidate valleys, one to the left of the valley and one to the right of the valley (and if the goal sector
     * is within the wide valley it is aso given as a candidate direction). A narrow valley gives one candidate
     * direction in the middle of the valley.
     *
     * @param candidateValleys the candidate valleys.
     * @param goalSector the sector number containing the goal.
     * @return a list of candidate directions (in sector numbers).
     */
    private ArrayList<Integer> getCandidateDirections(ArrayList<CandidateValley> candidateValleys, int goalSector) {

        ArrayList<Integer> candidateDirections = new ArrayList<Integer>();
        for (CandidateValley candidateValley: candidateValleys) {
            int leftBorderSector = candidateValley.getLeftBorderSector();
            int rightBorderSector = candidateValley.getRightBorderSector();
            if (candidateValley.isWideValley(S_MAX)) {
                candidateDirections.add(leftBorderSector - S_MAX /2);
                candidateDirections.add(rightBorderSector + S_MAX /2);
                if (leftBorderSector >= goalSector && rightBorderSector <= goalSector) {
                    candidateDirections.add(goalSector);
                }
            } else {
                candidateDirections.add((leftBorderSector + rightBorderSector) / 2);
            }
        }
        return candidateDirections;
    }

    /**
     * Creates a list of candidate valleys from the binary histogram.
     *
     * @param binaryHistogram the binary histogram.
     * @return
     */
    private ArrayList<CandidateValley> getCandidateValleys(Map<Integer, Integer> binaryHistogram) {
        Object[] objectKeys = binaryHistogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);
        ArrayList<CandidateValley> candidateValleys = new ArrayList<CandidateValley>();

        int index = 0;
        Integer startSector = null;
        while (index < keys.size()) {
            if (binaryHistogram.get(keys.get(index)) == 0) {
                if(startSector == null) {
                    startSector = keys.get(index);
                }
            } else {
                if(startSector != null) {

                    candidateValleys.add(new CandidateValley(keys.get(index - 1), startSector));
                    startSector = null;
                }
            }
            index++;
        }
        if (startSector != null) {

            candidateValleys.add(new CandidateValley(keys.get(keys.size() - 1), startSector));
        }
        return candidateValleys;
    }

    /**
     * Builds a binary histogram from a polar histogram. If the sector in the polar histogram is below the low
     * threshold, the corresponding value in the binary histogram is 0. If the sector in the polar histogram is above
     * the high threshold, the corresponding value in the binary histogram is 1. Otherwise, the value of the binary
     * is the same as the last sector in the binary histogram.
     *
     * @param histogram the polar histogram.
     * @return a created binary histogram.
     */
    private Map<Integer, Integer> buildBinaryHistogram(Map<Integer, Double> histogram) {
        Object[] objectKeys = histogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);

        Map<Integer, Integer> binaryHistogram = new HashMap<Integer, Integer>();
        boolean noObstacles = true;
        for (Integer key : keys) {
            double magnitude = histogram.get(key);
            if(magnitude > THRESHOLD_HIGH) {
                binaryHistogram.put(key, 1);
                noObstacles = false;
            } else if(magnitude < THRESHOLD_LOW) {
                binaryHistogram.put(key, 0);
            } else {
                Integer previousMagnitude = binaryHistogram.get(key - 1);
                binaryHistogram.put(key, (previousMagnitude == null) ? 0 : previousMagnitude);
            }
        }
        if (noObstacles) {
            return null;
        }
        return binaryHistogram;
    }


    /**
     * Builds a polar histogram where each sector gets the magnitude of the obstacle with the highest magnitude where
     * the obstacle is affecting the sector.
     *
     * @param distanceToCarrotPoint the distance to the carrot point.
     * @param distances the echoes from the laser sensor.
     * @return a polar histogram.
     */
    private  Map<Integer, Double> buildPolarHistogram(double distanceToCarrotPoint, double distances[]) {

        double d_max = distanceToCarrotPoint;
        Map<Integer, Double> sector_magnitudes = new HashMap<Integer, Double>();
        double b = A / d_max;
        double enlargement_angle;
        double direction_to_obstacle;

        int maxSector = NO_OF_SECTORS /2;
        if(NO_OF_SECTORS % 2 == 0) {
           maxSector = maxSector - 1;
        }
        for(int i = -NO_OF_SECTORS/2; i <= maxSector; i++) {
            sector_magnitudes.put(i, 0.0);
        }

        for(int i = 0; i < distances.length; i++) {
            if(distances[i] <= d_max) {
                double magnitude = A - (b*distances[i]);

                enlargement_angle = Math.asin(ENLARGEMENT_RADIUS / distances[i]);
                direction_to_obstacle = startAngle + (i*angleIncrement);

                int startSector = Math.max((int)((direction_to_obstacle - enlargement_angle) / angularResolution), - (NO_OF_SECTORS / 2));

                int endSector =  Math.min((int) ((direction_to_obstacle + enlargement_angle) / angularResolution) + 1, maxSector);

                for(int k = startSector; k <= endSector; k++) {
                    if (!sector_magnitudes.containsKey(k) || (sector_magnitudes.get(k) < magnitude)) {
                        sector_magnitudes.put(k, magnitude);
                    }
                }
            }
        }

        return sector_magnitudes;
    }

    private class CandidateValley {

        private int leftBorderSector;
        private int rightBorderSector;

        public CandidateValley(int leftBorderSector, int rightBorderSector) {
            this.leftBorderSector = leftBorderSector;
            this.rightBorderSector = rightBorderSector;
        }

        public int getRightBorderSector () {
            return rightBorderSector;
        }

        public int getLeftBorderSector() {
            return leftBorderSector;
        }

        public boolean isWideValley(int sMAX) {
            return (leftBorderSector - rightBorderSector) > sMAX;
        }
    }
}