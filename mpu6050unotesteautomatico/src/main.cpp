#include <Arduino.h> 
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empírico, mantém amostragem em ~200 Hz
const int NFast =  1000;    
const int NSlow = 10000;    
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

// --- DECLARAÇÕES DE FUNÇÕES
void ForceHeader();
void GetSmoothed();
void Initialize();
void SetOffsets(int TheOffsets[6]);
void ShowProgress();
void PullBracketsIn();
void PullBracketsOut();
void SetAveraging(int NewN);
// ------------------------------------------------------------------------

void ForceHeader() { 
    LinesOut = 99; 
}
    
void GetSmoothed() { 
    int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++) { Sums[i] = 0; }

    for (i = 1; i <= N; i++) {
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
    }
    for (i = iAx; i <= iGz; i++) { 
        Smoothed[i] = (Sums[i] + N/2) / N ; 
    }
}

void Initialize() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200); // Alterado para bater com o monitor_speed do seu .ini

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
    
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    Serial.println("\nat 600 Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("700 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("800 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("900 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();    
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("1000 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
}

void SetOffsets(int TheOffsets[6]) { 
    accelgyro.setXAccelOffset(TheOffsets [iAx]);
    accelgyro.setYAccelOffset(TheOffsets [iAy]);
    accelgyro.setZAccelOffset(TheOffsets [iAz]);
    accelgyro.setXGyroOffset (TheOffsets [iGx]);
    accelgyro.setYGyroOffset (TheOffsets [iGy]);
    accelgyro.setZGyroOffset (TheOffsets [iGz]);
}

void ShowProgress() { 
    if (LinesOut >= LinesBetweenHeaders) { 
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
    }
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++) { 
        Serial.print(LBRACKET);
        Serial.print(LowOffset[i]);
        Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz) { 
            Serial.println(RBRACKET); 
        } else { 
            Serial.print("]\t"); 
        }
    }
    LinesOut++;
}

void PullBracketsIn() { 
    boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) { 
        StillWorking = false;
        if (AllBracketsNarrow && (N == NFast)) { 
            SetAveraging(NSlow); 
        } else { 
            AllBracketsNarrow = true; 
        }
        for (int i = iAx; i <= iGz; i++) { 
            if (HighOffset[i] <= (LowOffset[i]+1)) { 
                NewOffset[i] = LowOffset[i]; 
            } else { 
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10)) { 
                    AllBracketsNarrow = false; 
                }
            }
        }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++) { 
            if (Smoothed[i] > Target[i]) { 
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
            } else { 
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
            }
        }
        ShowProgress();
    }
}

void PullBracketsOut() { 
    boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();
 
    while (!Done) { 
        Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++) { 
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i]) { 
                Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
            } else { 
                NextLowOffset[i] = LowOffset[i]; 
            }
        }
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++) { 
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i]) { 
                Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
            } else { 
                NextHighOffset[i] = HighOffset[i]; 
            }
        }
        ShowProgress();
        for (int i = iAx; i <= iGz; i++) { 
            LowOffset[i] = NextLowOffset[i];   
            HighOffset[i] = NextHighOffset[i]; 
        }
     }
}

void SetAveraging(int NewN) { 
    N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
}

void setup() { 
    Initialize();
    for (int i = iAx; i <= iGz; i++) { 
        Target[i] = 0; 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
    }
    Target[iAz] = 16384; // Alvo padrão de 1g para o eixo Z do acelerômetro
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
    
    Serial.println("-------------- done --------------");
}

void loop() {
    // Fica vazio pois a calibração roda apenas uma vez no setup
}