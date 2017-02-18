package com.team254.lib.util;


import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;

public class CSVWriter {
    private int mColumns;
    private ArrayList<String> mList;
    DecimalFormat mDf = new DecimalFormat("#.000");
    PrintWriter mOutput = null;

    public CSVWriter(String fileName, String[] columnNames) {
        mColumns = columnNames.length;
        mList = new ArrayList<>(mColumns);
        try {
            mOutput = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        for (int i = 0; i < columnNames.length; ++i) {
            addValue(i, columnNames[i]);
        }
        write();
    }

    public void addValue(int columnIndex, double value) {
        addValue(columnIndex, mDf.format(value));
    }

    public void addValue(int columnIndex, String value) {
        if (columnIndex > -1 && columnIndex < mColumns) {
            mList.add(columnIndex, value);
        }
    }

    public String toRowString() {
        StringBuffer sb = new StringBuffer();
        for (int i = 0; i < mColumns; i++) {
            String num = mList.get(i);
            if (i > 0) {
                sb.append(", ");
            }
            sb.append(num);
        }
        return sb.toString();
    }

    public void write() {
        String nextLine = toRowString();
        for (int i = 0; i < mColumns; i++) {
            mList.add(i, "0.000");
        }
        if (mOutput != null) {
            mOutput.println(nextLine);
        }
    }

    public void flush() {
        if (mOutput != null) {
            mOutput.flush();
        }
    }

}
