package com.ailab.controllers;

import com.ailab.tools.DifferentialDriveRequest;
import com.ailab.tools.Request;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;

/**
 * Created by Jonas on 2014-09-15.
 */
public class Robot {

    String host;
    int port;
    ObjectMapper mapper;

    public Robot(String host, int port) {
        this.host = host;
        this.port = port;
        mapper = new ObjectMapper();
    }

    public void drive(double linearSpeed, double angularSpeed) throws Exception {
        DifferentialDriveRequest dr = new DifferentialDriveRequest();
        dr.setAngularSpeed(Math.PI * angularSpeed);
        dr.setLinearSpeed(linearSpeed);
        int rc = putRequest(dr);
    }

    public int putRequest(Request r) throws Exception
    {
        URL url = new URL(host + ":" + port + r.getPath());

        HttpURLConnection connection = (HttpURLConnection)url.openConnection();

        connection.setDoOutput(true);

        connection.setRequestMethod("POST");
        connection.setRequestProperty("Content-Type", "application/json");
        connection.setUseCaches (false);

        OutputStreamWriter out = new OutputStreamWriter(
                connection.getOutputStream());

        // construct a JSON string
        String json = mapper.writeValueAsString(r.getData());

        // write it to the web server
        out.write(json);
        out.close();

        // wait for response code
        int rc = connection.getResponseCode();

        return rc;
    }
}
