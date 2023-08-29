#include <Arduino.h>

#include "head-tracker.hpp"

HeadTracker tracker;

void setup() {
    tracker.begin();
}

void loop() {
    tracker.tick();
    delay(300);
}