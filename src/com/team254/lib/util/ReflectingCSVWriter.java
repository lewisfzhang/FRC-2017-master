package com.team254.lib.util;

import java.lang.reflect.Field;

public class ReflectingCSVWriter<T> extends CSVWriter {
    Field[] mFields;
    
    static <T> String[] getFieldNames(Class<T> typeClass) {
        Field[] fields = typeClass.getFields();
        String[] names = new String[fields.length];
        for (int i = 0; i < names.length; ++i) {
            names[i] = fields[i].getName();
        }
        return names;
    }
    
    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        super(fileName, getFieldNames(typeClass));
        mFields = typeClass.getFields();
    }
    
    public void writeLine(T value) {
        for (Field field : mFields) {
            try {
                addValue(field.getName(), field.get(value).toString());
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        write();
    }
}
