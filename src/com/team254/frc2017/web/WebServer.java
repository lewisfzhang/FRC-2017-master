package com.team254.frc2017.web;

import com.team254.frc2017.web.handlers.ConstantsServlet;
import com.team254.frc2017.web.handlers.ResetConstantsServlet;
import com.team254.lib.util.TaskQueue;

import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.servlet.DefaultServlet;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.servlet.ServletHolder;

import java.util.ArrayList;

public class WebServer {
    private static Server server;
    private static ArrayList<StateStreamSocket> updateStreams = new ArrayList<StateStreamSocket>();
    private static TaskQueue streamUpdate = new TaskQueue(200);

    public static void main(String args[]) {

        startServer();
        while (true) {
            try {
                Thread.sleep(1000000);
            } catch (Exception e) {
            }
        }

    }

    public static void startServer() {
        if (server != null) {
            return;
        }
        server = new Server(5800);
        ServletContextHandler context = new ServletContextHandler(ServletContextHandler.SESSIONS);
        context.setContextPath("/");
        server.setHandler(context);

        ServletHolder constantsHolder = new ServletHolder("constants", new ConstantsServlet());
        context.addServlet(constantsHolder, "/constants");

        ServletHolder resetConstantsHolder = new ServletHolder("resetConstants", new ResetConstantsServlet());
        context.addServlet(resetConstantsHolder, "/resetconstants");

        ServletHolder holderPwd = new ServletHolder("default", new DefaultServlet());

        holderPwd.setInitParameter("dirAllowed", "true");
        context.addServlet(holderPwd, "/");

        Thread serverThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    server.start();
                    server.join();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        serverThread.setPriority(Thread.MIN_PRIORITY);
        serverThread.start();
        streamUpdate.start();
    }

    public static void registerStateStreamSocket(StateStreamSocket s) {
        updateStreams.add(s);
    }

    public static void unregisterStateStreamSocket(StateStreamSocket s) {
        updateStreams.remove(s);
    }

    public static Runnable updateRunner = new Runnable() {
        public void run() {
            for (int i = 0; i < updateStreams.size(); ++i) {
                StateStreamSocket s = updateStreams.get(i);
                if (s != null && s.isConnected() && !s.canBeUpdated()) {
                } else if ((s == null || !s.isConnected() || !s.update()) && i < updateStreams.size()) {
                    updateStreams.remove(i);
                }
            }
        }
    };

    public static void updateAllStateStreams() {
        boolean runUpdate = false;
        for (StateStreamSocket s : updateStreams) {
            runUpdate = (s != null && s.canBeUpdated());
            if (runUpdate) {
                break;
            }
        }
        if (runUpdate) {
            streamUpdate.addTask(updateRunner);
        }
    }
}