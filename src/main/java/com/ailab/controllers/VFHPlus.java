package com.ailab.controllers;

/**
 * Created by Jonas on 2014-09-22.
 */
public class VFHPlus {

    public static final double a = 1.0;
    public static final double ROBOT_RADIUS = 0.003;
    public static final double d_s = 0.001;
    public static final double ENLARGEMENT_RADIUS = ROBOT_RADIUS + d_s;
    public static final int NO_OF_SECTORS = 45;
    public static final int READS_PER_SECTOR = 270 / NO_OF_SECTORS;

    public double[] buildPolarHistogram(double distanceToCarrotPoint, double distances[], double angle_increment, double start_angle, double vehicle_angle) {

        double d_max = distanceToCarrotPoint;
        double sector_magnitude[] = new double[NO_OF_SECTORS];
        double b = a / d_max;
        double enlargement_angle;
        double direction_to_obstacle;

        double magnitudes[] = new double[distances.length];

        for(int i = 0; i < magnitudes.length; i++) {
            if(distances[i] <= d_max) {
                magnitudes[i] = a - (b*distances[i]);
                enlargement_angle = Math.asin(ENLARGEMENT_RADIUS / distances[i]);
                direction_to_obstacle = start_angle + (i*angle_increment);

                double min_obstacle_angle = direction_to_obstacle - enlargement_angle;
                double max_obstacle_angle = direction_to_obstacle + enlargement_angle;

                double sector_min_angle;
                double sector_max_angle;
                for(int k = 0; k < NO_OF_SECTORS; k++) {
                    sector_min_angle = k*READS_PER_SECTOR*angle_increment;
                    sector_max_angle = (k+1)*READS_PER_SECTOR*angle_increment;

                    
                }
            }
        }

        return null;
    }

    public double[] buildPolarHistogram2(double distanceToCarrotPoint, double distances[], double angle_increment, double start_angle, double vehicle_angle) {

        double d_max = distanceToCarrotPoint;
        double polarHistogram[] = new double[NO_OF_SECTORS];
        double b = a / d_max;
        double enlargement_angle[] = new double[distances.length];
        double direction_to_obstacle[] = new double[distances.length];

        double magnitudes[] = new double[distances.length];

        for(int i = 0; i < magnitudes.length; i++) {
            magnitudes[i] = a - (b*distances[i]);
            enlargement_angle[i] = Math.asin(ENLARGEMENT_RADIUS / distances[i]);
            direction_to_obstacle[i] = start_angle + (i*angle_increment) + vehicle_angle;
        }

        int h_prime = 0;
        double max_magnitude = 0.0;
        double alpha = READS_PER_SECTOR * angle_increment;

        for(int k = 0; k < NO_OF_SECTORS; k++) {
            max_magnitude = 0.0;
            h_prime = 0;
            for(int j = 0; j < READS_PER_SECTOR; j++) {
                if((max_magnitude < magnitudes[k*READS_PER_SECTOR + j]) && (distances[k*READS_PER_SECTOR + j] < d_max)) {
                    max_magnitude = magnitudes[k*READS_PER_SECTOR + j];

                    if((k*alpha >= direction_to_obstacle[k*READS_PER_SECTOR + j] - enlargement_angle[k*READS_PER_SECTOR + j]) ||
                            (k*alpha <= direction_to_obstacle[k*READS_PER_SECTOR + j] + enlargement_angle[k*READS_PER_SECTOR + j])) {
                        h_prime = 1;
                    } else {
                        h_prime = 0;
                    }
                }
            }
            polarHistogram[k] = max_magnitude * h_prime;
        }


        return null;
    }
}
