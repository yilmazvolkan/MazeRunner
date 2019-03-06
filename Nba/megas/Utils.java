package megas;

import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.robotics.LightDetectorAdaptor;
import lejos.robotics.SampleProvider;

public class Utils {
	
	// =================================================================
	// ========================== THRESHOLDS ===========================
	// =================================================================			
	public static final double BLUE_BALL_THRESHOLD = 0.37;
	
	public static final int BLUE_BALL_CODE = 1;
	public static final int RED_BALL_CODE = 0;
	
	// =================================================================
	// ========================== ENUMS ================================
	// =================================================================		
	public static final int NXT_RED_MODE = 0;
	
	static NXTLightSensor nxtLightSensor;
	static LightDetectorAdaptor nxtLightDetectorAdaptor;
	
	public Utils() {
		nxtLightSensor = new NXTLightSensor(SensorPort.S2);
		nxtLightDetectorAdaptor = new LightDetectorAdaptor((SampleProvider)nxtLightSensor);
		
		nxtLightSensor.setCurrentMode("Red");
    	nxtLightSensor.setFloodlight(NXT_RED_MODE);
    	nxtLightSensor.setFloodlight(true);
    	nxtLightDetectorAdaptor.setReflected(true);
	}
	
	public static int determineBallColor(GraphicsLCD graphicsLCD) {
		
		double reading = nxtLightDetectorAdaptor.getLightValue();
		if (reading < BLUE_BALL_THRESHOLD) {
			graphicsLCD.clear();
			graphicsLCD.drawString("BLUE BALL", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
			graphicsLCD.refresh();
			return BLUE_BALL_CODE;
		} else {
			graphicsLCD.clear();
			graphicsLCD.drawString("RED BALL", graphicsLCD.getWidth()/2, 0, GraphicsLCD.VCENTER|GraphicsLCD.HCENTER);
			graphicsLCD.refresh();
			return RED_BALL_CODE;
		}
	}
}
