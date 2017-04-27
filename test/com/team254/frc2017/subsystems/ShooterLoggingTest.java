package com.team254.frc2017.subsystems;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.lib.util.ReflectingCSVWriter;

public class ShooterLoggingTest {

    @Test
    public void test() {
        ReflectingCSVWriter<Shooter.ShooterDebugOutput> writer = new ReflectingCSVWriter<>("test.csv", Shooter.ShooterDebugOutput.class);
        Shooter.ShooterDebugOutput debug = new Shooter.ShooterDebugOutput();
        debug.control_method = Shooter.ControlMethod.HOLD;
        writer.add(debug);
        writer.write();
        writer.flush();
    }

}
