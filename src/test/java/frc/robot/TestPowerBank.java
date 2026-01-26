package frc.robot;

import static frc.robot.Power.PowerBank.centralBank;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Power.PowerBroker;

public class TestPowerBank {
    private static PowerBroker broker1 = new PowerBroker(() -> 1.0);
    private static PowerBroker broker2 = new PowerBroker(() -> 2.0);
    private static double powerUnit = centralBank.getMaxPower() / 3.0;

    @BeforeEach
    void testResetBrokers() {
        broker1.releasePower();
        broker2.releasePower();
    }

    @Test
    void testUnderPower() {
        // Just two power units is below the limit so should be provided in full
        double p1 = broker1.requestPower(powerUnit);
        double p2 = broker2.requestPower(powerUnit);
        assertEquals(powerUnit, p1);
        assertEquals(powerUnit, p2);
    }

    @Test
    void testOverPower() {
        // Let's try to get a lot of power
        double p1 = broker1.requestPower(10.0 * powerUnit);
        double p2 = broker2.requestPower(10.0 * powerUnit);
        assertTrue(p1 <= centralBank.getMaxPower());
        assertTrue(p2 <= centralBank.getMaxPower());
        // System.out.println("Returned power: p1="+p1+", p2="+p2);
        p1 = broker1.requestPower(10.0 * powerUnit);
        // Broker One has priority = 1.0 so should get only one power unit
        assertTrue(p1 <= powerUnit);
        // System.out.println("Returned power: p1="+p1+", p2="+p2);
        assertTrue(p1+p2 <= centralBank.getMaxPower());
    }

    @Test
    void testDiscrete() {
        double p2 = broker2.requestPower(powerUnit * 2);
        double p1 = broker1.requestPower(powerUnit * 2, powerUnit * 2);
        // System.out.println("Returned power: p1="+p1+", p2="+p2);
        p2 = broker2.requestPower(powerUnit * 2);
        // System.out.println("Returned power: p1="+p1+", p2="+p2);
        assertEquals(0, p1);
        assertEquals(powerUnit*2, p2);
    }

    // This stress test only makes sense if run on the robot hardware
    // Uncomment this @Test decorator and deploy to RoboRIO if you know how to run it there
    // @Test
    void testStress() {
        List<PowerBroker> brokers = new ArrayList<>();
        for (int i = 0; i < 500; i++) {
            PowerBroker pb = new PowerBroker(() -> 1.0);
            brokers.add(pb);
        }
        Timer stopwatch = new Timer();
        stopwatch.restart();
        for (PowerBroker pb: brokers) {
            pb.requestPower(42);
        }
        double elapsed = stopwatch.get();
        stopwatch.stop();
        System.out.println("Time elapsed: "+elapsed);
        for (PowerBroker pb: brokers) {
            pb.releasePower();
        }
    }
}
