package com.ailab.controllers;

import com.ailab.tools.DifferentialDriveRequest;
import com.ailab.tools.LocalizationResponse;
import com.ailab.tools.Request;
import com.ailab.tools.Response;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.util.Map;

/**
 * An object of this class contains information about the connection to the robot in MRDS, it sends and receives
 * JSON objects to/from the MRDS server and drives the robot at specified speed and angular speed.
 *
 * Created by Jonas on 2014-09-15.
 */
public class Robot {

    private final String host;
    private final int port;
    private final ObjectMapper mapper;

    public Robot(String host, int port) {
        this.host = host;
        this.port = port;
        mapper = new ObjectMapper();
    }

    /**
     * Send a differential drive request to MRDS which sets the speed of the vehicle.
     * @param linearSpeed New linear speed of the vehicle.
     * @param angularSpeed New Angular speed of the vehicle.
     * @throws Exception
     */
    public void drive(double linearSpeed, double angularSpeed) throws Exception {
        DifferentialDriveRequest dr = new DifferentialDriveRequest();
        dr.setAngularSpeed(angularSpeed);
        dr.setLinearSpeed(linearSpeed);
        int rc = putRequest(dr);
    }

    /**
     * Sends a request to the MRDS server.
     * @param r the request to be sent.
     * @return the response code.
     * @throws Exception
     */
    public int putRequest(Request r) throws Exception {
        URL url = new URL(host + ":" + port + r.getPath());

        HttpURLConnection connection = (HttpURLConnection) url.openConnection();

        connection.setDoOutput(true);

        connection.setRequestMethod("POST");
        connection.setRequestProperty("Content-Type", "application/json");
        connection.setUseCaches(false);

        OutputStreamWriter out = new OutputStreamWriter(
                connection.getOutputStream());

        // construct a JSON string
        String json = mapper.writeValueAsString(r.getData());

        // write it to the web server
        out.write(json);
        out.close();

        // wait for response code
        return connection.getResponseCode();
    }

    /**
     * Get a response from the robot
     *
     * @param r response to fill in
     * @return response same as parameter
     * @throws Exception
     */
    public Response getResponse(Response r) throws IOException {
        URL url = new URL(host + ":" + port + r.getPath());
        //System.out.println(url);

        // open a connection to the web server and then get the resulting data
        URLConnection connection = url.openConnection();
        BufferedReader in = new BufferedReader(new InputStreamReader(
                connection.getInputStream()));

        // map it to a Java Map
        Map<String, Object> data = mapper.readValue(in, Map.class);
        r.setData(data);

        in.close();

        return r;
    }

    /**
     * Returns the bearing angle of the vehicle.
     * @param lr the localization response containing the orientation.
     * @return bearing angle.
     */
    double getBearingAngle(LocalizationResponse lr) {
        double e[] = lr.getOrientation();

        double angle = 2 * Math.atan2(e[3], e[0]);
        return angle * 180 / Math.PI;
    }
}
