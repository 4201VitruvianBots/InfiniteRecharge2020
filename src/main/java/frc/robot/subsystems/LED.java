/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/*
Subsystem for controlling robot LEDs
 */

public class LED extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private final AddressableLED LEDStrip;
    private final AddressableLEDBuffer LEDBuffer;
    int start = (int) Timer.getFPGATimestamp() * 5;

    int stripLength;
    double rainbows = 3;
    double speed = 8;
    ColorSensor m_colorSensor;
    double hueOffset = 0;
    int head = 0;
    int offset = 0;
    int cRed = 0;
    int cGreen = 0;
    int cBlue = 0;
    int realPanelColor = 0;
    int state = - 1;
    private int red, green, blue;

    public LED(ColorSensor colorSensor) {
        // Setup LED strip
        m_colorSensor = colorSensor;
        LEDStrip = new AddressableLED(Constants.ledPort);
        LEDBuffer = new AddressableLEDBuffer(111);
        LEDStrip.setLength(LEDBuffer.getLength());
        LEDStrip.setData(LEDBuffer);
        LEDStrip.start();
        stripLength = LEDBuffer.getLength() / 2;
//    LEDStripB.setLength(LEDBuffer.getLength());
//    LEDStripB.setData(LEDBuffer);
//    LEDStripB.start();

        red = 0;
        green = 125;
        blue = 0;
        setSolidColor();
//    SmartDashboard.putNumber("Rainbows", rainbows);
//    SmartDashboard.putNumber("Speed", speed);
  }
  // Set LED color based on RGB value

    public void setRGB(int red, int green, int blue) {
        this.red = (int) (red * 0.5);
        this.blue = (int) (blue * 0.5);
        this.green = (int) (green * 0.5);
    }

    // Set LED color ???
    public void setSolidColor(){
        for(int i = 0; i < LEDBuffer.getLength(); i++){
            LEDBuffer.setRGB(i, red, green, blue);
        }
    }
    public void resetLED() {
        for(int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setBlinkingColor(boolean blinkType) {
        double time = (int) (5 * Timer.getFPGATimestamp());
        if(! blinkType) {
            if(time / 2 == Math.floor(time / 2)) {
                for(int i = 0; i < stripLength; i++) {
                    LEDBuffer.setRGB(i, red, green, blue);
                    LEDBuffer.setRGB(stripLength + i, red, green, blue);
                }
            } else
                resetLED();
        } else {
            resetLED();
            for(int i = head; i < LEDBuffer.getLength(); i += 2) {
                LEDBuffer.setRGB(i % LEDBuffer.getLength(), red, green, blue);
            }
            Timer.delay(0.2);
            head = ++ head % 2;
        }
    }

    public void setRainbow(double iterations, double speed) {
        for(int i = 0; i < stripLength; i++) {
            LEDBuffer.setHSV(i, (int) (180 * iterations * i / stripLength + hueOffset) % 180, 255, 255);
            LEDBuffer.setHSV(stripLength + i, (int) (180 * iterations * i / stripLength + hueOffset) % 180, 255, 255);
        }
        hueOffset = (hueOffset + 3 * speed * iterations) % 180;
        Timer.delay(0.05);
    }

    public void trail(int interval) {
        resetLED();
        for(int i = head; i < LEDBuffer.getLength(); i += interval) {
            LEDBuffer.setRGB(i % LEDBuffer.getLength(), red, green, blue);
        }
        Timer.delay(0.03);
        head = ++ head % interval;
    }

    public void flash() {
        setSolidColor();
        Timer.delay(0.1);
        resetLED();
        Timer.delay(0.1);
        setSolidColor();
        Timer.delay(0.25);
    }

    public void colorToRGB(int color) {
        switch(color) {
            case 1:
                setRGB(255, 0, 0);
                break;
            case 2:
                setRGB(0, 255, 0);
                break;
            case 3:
                setRGB(0, 255, 255);
                break;
            case 4:
                setRGB(255, 255, 0);
                break;
        }
    }

    public void colorWheel() {
    /*for(int i = 0; i < LEDBuffer.getLength(); i = (int) (i + LEDBuffer.getLength() * 0.125)) {
      colorToRGB((m_colorSensor.panelColor() + i) % 4);
      for(int ii = i; ii < (int) (i + LEDBuffer.getLength() * 0.125); ii++)
        LEDBuffer.setRGB(ii, red, green, blue);
    }*/

        if(m_colorSensor.panelColor() != 0)
            realPanelColor = m_colorSensor.panelColor();
        for(int i = 0; i < 7; i++) {
            switch((realPanelColor + i) % 4 + 1) {
                case 1:
                    cRed = 255;
                    cGreen = 0;
                    cBlue = 0;
                    break;
                case 2:
                    cRed = 0;
                    cGreen = 255;
                    cBlue = 0;
                    break;
                case 3:
                    cRed = 0;
                    cGreen = 255;
                    cBlue = 255;
                    break;
                case 4:
                    cRed = 255;
                    cGreen = 255;
                    cBlue = 0;
                    break;
            }
            for(int ii = (int) Math.floor(i * LEDBuffer.getLength() / 7); ii < (int) Math.floor((i + 1) * LEDBuffer.getLength() / 7); ii++) {
                LEDBuffer.setRGB(ii, cRed, cGreen, cBlue);
            }
        }
    }

    public void setLED() {
        switch(state) {
            case 0:
                setRainbow(3, 8);
                break;
            case 1:
                setRGB(255, 200, 0); // Blinking dirty-yellow
                setBlinkingColor(true);
                break;
            case 2:
                setRGB(255, 0, 0); // Solid red
                setSolidColor();
                break;
            case 3:
                setRGB(255, 255, 255); // Flashing white
                flash();
                break;
            case 4:
                setRGB(66, 194, 23); // Trailing vitruvian green
                trail(5);
                break;
            case 5:
                setRGB(0, 110, 255); // Blinking dark blue
                setBlinkingColor(true);
                break;
            case 6:
                setRGB(255, 110, 0); // Trailing reddish-orange
                trail(8);
                break;
            case 7:
                setRGB(255, 0, 0); // Blinking red
                setBlinkingColor(true);
                break;
            case 8:
                setRGB(0, 255, 0); // Blinking green
                setBlinkingColor(true);
                break;
            case 9:
                colorWheel();
                break;
            case 10: // Shoot on the Move: able to shoot and hit outer
                setRGB(0, 255, 0); // Solid green
                setSolidColor();
                break;
            default:
                setRGB(106, 90, 205); // Blinking purple
                setBlinkingColor(true);
        }
        LEDStrip.setData(LEDBuffer);
    }

    public void setState(int state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //setRainbow(rainbows, speed);
    /*rainbows = SmartDashboard.getNumber("Rainbows", 0);
    speed = SmartDashboard.getNumber("Speed", 0);
    setBuffer();*/
    if(RobotBase.isReal()) {
      setLED();
      LEDStrip.setData(LEDBuffer);
    }
  }
}
