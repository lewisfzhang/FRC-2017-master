package pixy;

import edu.wpi.first.wpilibj.SPI;

public class SpiLogger {
    private SPI spi;

    public SpiLogger() {
        spi = new SPI(SPI.Port.kOnboardCS0);
        // spi = new SPI(SPI.Port.kMXP);
        spi.setMSBFirst();
        spi.setClockActiveHigh();
        spi.setSampleDataOnRising();
        spi.setChipSelectActiveLow();
        spi.setClockRate(500000);
    }

    public byte getByte(byte data) {
        byte[] send = new byte[] { data };
        byte[] recv = new byte[send.length];
        spi.transaction(send, recv, recv.length);
        return recv[0];
    }

    public int getWord() {
        int data = 0;
        int msb = Byte.toUnsignedInt(getByte((byte) 0));
        int lsb = Byte.toUnsignedInt(getByte((byte) 0));
        msb <<= 8;
        data = msb | lsb;
        return data;
    }

    public void startLogging() throws Exception {
        while (true) {
            int data = getWord();
            System.out.println(Integer.toString(data, 16));
            Thread.sleep(10);
        }
    }
}
