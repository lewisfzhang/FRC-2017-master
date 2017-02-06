package com.team254.frc2017.web.handlers;

import com.team254.frc2017.Constants;
import com.team254.lib.util.ConstantsBase.Constant;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Collection;

public class ResetConstantsServlet extends HttpServlet {

    private void buildPage(HttpServletResponse response) throws IOException {
        Constants constants = new Constants();

        response.setContentType("text/html");
        response.setStatus(HttpServletResponse.SC_OK);
        PrintWriter out = response.getWriter();

        out.println("<html>");
        out.println("<body>");
        out.println(
                "<h1>Constants file truncated</h1><p>Constants have been reset to the defaults as defined in the code</p><a href='/constants'>Back</a>");
        out.println("</body>");
        out.println("</html>");
    }

    @Override
    protected void doGet(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException {
        Constants constants = new Constants();
        constants.truncateUserConstants();

        buildPage(response);
    }
}