
import java.util.ArrayList;
import java.util.Random;

import com.embeddedunveiled.serial.SerialComManager;
import com.embeddedunveiled.serial.SerialComManager.BAUDRATE;
import com.embeddedunveiled.serial.SerialComManager.DATABITS;
import com.embeddedunveiled.serial.SerialComManager.FLOWCONTROL;
import com.embeddedunveiled.serial.SerialComManager.PARITY;
import com.embeddedunveiled.serial.SerialComManager.STOPBITS;

public class BaudTest {
  static long start, stop, elapsed, errorCount;
  static BAUDRATE baudrate = BAUDRATE.B115200;
  static Random random = new Random();
  static int byteCount = 1000;
  static byte[] buffer = new byte[byteCount];
  static ArrayList<Byte> mirrorBuffer = new ArrayList<Byte>();
  static SerialComManager scm;
  static long handle;

  static void init() throws Exception {
    scm = new SerialComManager();
    handle = scm.openComPort("/dev/cu.wchusbserial1420", true, true, false);
    scm.configureComPortData(handle, DATABITS.DB_8, STOPBITS.SB_1, PARITY.P_NONE, baudrate, 0);
    scm.configureComPortControl(handle, FLOWCONTROL.NONE, 'x', 'x', false, false);
    Thread.sleep(2000);
  }

  static void print() {
    System.out.println("error count is: " + errorCount);
    System.out.println("elapsed time (ms): " + elapsed);                        
    System.out.println("measured datarate (bits/s): " + (2 * byteCount * 8 / (float) elapsed * 1000));
    System.out.println("used datarate (baud): " + baudrate.getValue());
  }

  static void singleByteTest() throws Exception {
    errorCount = 0;
    start = System.currentTimeMillis();
    for (int i = 0; i < byteCount; i++) {
      byte t = (byte) (i % 100); // byte values 0 - 99
      scm.writeSingleByte(handle, t);
      if (scm.readSingleByte(handle)[0] != t)
        errorCount++;
    }
    stop = System.currentTimeMillis();
    elapsed = stop - start;
    print();
  }

  static void bufferTest() throws Exception {
    for (int i = 0; i < buffer.length; i++) {
      buffer[i] = (byte) (random.nextInt(100) + 100); // byte values 100-199
    }
    errorCount = 0;
    start = System.currentTimeMillis();
    scm.writeBytes(handle, buffer);
    scm.writeSingleByte(handle, (byte) 255); // send buffer signal
    byte[] readBuffer;
    while ((readBuffer = scm.readBytes(handle, byteCount)) != null)
      for (byte b : readBuffer) {
        mirrorBuffer.add(b);
      }
    stop = System.currentTimeMillis();
    elapsed = stop - start;
    if (mirrorBuffer.size() != byteCount)
      System.out.println("ERROR: read buffer length: " + mirrorBuffer.size());
    for (int i = 0; i < mirrorBuffer.size(); i++) {
      if (mirrorBuffer.get(i) != buffer[i])
        errorCount++;
    }
    elapsed = stop - start;
    print();
  }

  public static void main(String[] args) {
    try {
      init();
      singleByteTest();
      bufferTest();
      scm.closeComPort(handle);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
