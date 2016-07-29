#include <microsmooth.h>

uint16_t *ptr;

void setup()
{
    Serial.begin(115200);
    pinMode(3, INPUT);
    ptr = ms_init(EMA);
    if(ptr == NULL) Serial.println("No memory");
}

void loop()
{
    int i = pulseIn(3, HIGH, 25000);
    Serial.print("Input: "); Serial.println(i);
    Serial.println(sga_filter(i, ptr));
    Serial.println("Back");
    delay(100);
}


