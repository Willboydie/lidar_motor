#include "garmin_hardware_test.hpp"

LIDARLite_v3HP myLidarLite;
// GarminV3HP myGarminV3HP = GarminV3HP(GARMIN_SCL, GARMIN_SDA);

void PrintMenu(void) {
    Serial.println("=====================================");
    Serial.println("== Type a single character command ==");
    Serial.println("=====================================");
    Serial.println(" S - Single Measurement");
    Serial.println(" C - Continuous Measurement");
    Serial.println(" T - Timed Measurement");
    Serial.println(" . - Stop Measurement");
    Serial.println(" D - Dump Correlation Record");
    Serial.println(" P - Peak Stack Example");
}


uint8_t distanceSingle(uint16_t * distance)
{
    myLidarLite.waitForBusy();   // ensure idle before triggering
    myLidarLite.takeRange();     // trigger
    myLidarLite.waitForBusy();   // wait for completion
    *distance = myLidarLite.readDistance();
    return 1;
}


uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 0;
    if (myLidarLite.getBusyFlag() == 0)
    {
        myLidarLite.takeRange();          // trigger next
        // myGarminV3HP.requestDistance();
        // myGarminV3HP.readLatestDistance();
        *distance = myLidarLite.readDistance(); // read previous
        newDistance = 1;
    }
    return newDistance;
}


uint8_t distanceFast(uint16_t * distance)
{
    myLidarLite.waitForBusy();   // wait until idle
    myLidarLite.takeRange();     // trigger
    *distance = myLidarLite.readDistance(); // read previous result
    return 1;
}


void dumpCorrelationRecord()
{
    myLidarLite.correlationRecordToSerial(256);
}


void peakStackExample() {
    int16_t   peakArray[8];
    int16_t   distArray[8];
    uint8_t   i;

    myLidarLite.peakStackRead(peakArray, distArray);

    Serial.println();
    Serial.println("IDX PEAK DIST");
    for (i=0 ; i<8 ; i++)
    {
        Serial.print(i);
        Serial.print("   ");
        Serial.print(peakArray[i]);
        Serial.print("  ");
        Serial.print(distArray[i]);
        Serial.println();
    }
}


void garmin_hardware_test() {


    // Initialize Arduino serial port (for display of ASCII output to PC)
    Serial.begin(115200);

    // Initialize Arduino I2C (for communication to LidarLite)
#if defined(STM32G4xx)
    Wire.setSCL(GARMIN_SCL);
    Wire.setSDA(GARMIN_SDA);
#endif
    Wire.begin();
    #ifdef FAST_I2C
        #if ARDUINO >= 157
            Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
        #endif
    #endif

    // Configure the LidarLite internal parameters so as to lend itself to
    // various modes of operation by altering 'configure' input integer to
    // anything in the range of 0 to 5. See LIDARLite_v3HP.cpp for details.
    myLidarLite.configure(0);
    // myGarminV3HP.enable();
    // myGarminV3HP.init();

    uint16_t distance;
    uint8_t  newDistance = 0;
    uint8_t  c;
    rangeType_T rangeMode = RANGE_NONE;

    PrintMenu();

    // Continuous loop
    while (1) {
        // Each time through the loop, look for a serial input character
        if (Serial.available() > 0) {
            //  read input character ...
            c = (uint8_t) Serial.read();

            // ... and parse
            switch (c)
            {
                case 'S':
                case 's':
                    rangeMode = RANGE_SINGLE;
                    break;

                case 'C':
                case 'c':
                    rangeMode = RANGE_CONTINUOUS;
                    break;

                case 'T':
                case 't':
                    rangeMode = RANGE_TIMER;
                    break;

                case '.':
                    rangeMode = RANGE_NONE;
                    break;

                case 'D':
                case 'd':
                    rangeMode = RANGE_NONE;
                    dumpCorrelationRecord();
                    break;

                case 'P':
                case 'p':
                    rangeMode = RANGE_NONE;
                    peakStackExample();
                    break;

                case 0x0D:
                case 0x0A:
                    break;

                default:
                    rangeMode = RANGE_NONE;
                    PrintMenu();
                    break;
            }
        }

        switch (rangeMode)
        {
            case RANGE_NONE:
                newDistance = 0;
                break;

            case RANGE_SINGLE:
                newDistance = distanceSingle(&distance);
                break;

            case RANGE_CONTINUOUS:
                newDistance = distanceContinuous(&distance);
                break;

            case RANGE_TIMER:
                delay(250); // 4 Hz
                newDistance = distanceFast(&distance);
                break;

            default:
                newDistance = 0;
                break;
        }

        // When there is new distance data, print it to the serial port
        if (newDistance)
        {
            Serial.println(distance);
        }

        // Single measurements print once and then stop
        if (rangeMode == RANGE_SINGLE)
        {
            rangeMode = RANGE_NONE;
        }
    }
}