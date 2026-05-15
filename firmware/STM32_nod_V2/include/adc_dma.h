#pragma once
#include <stdint.h>

// ADC modul az arammero csatornakhoz.
//
// analogRead alapu implementacio (nem DMA).
// 3 csatorna x ~2 us = ~6 us blokkolasvaltozat per mintavetel (5 ms-onkent).
//
// Csatornasorrend:
//   [0] = Ia  (PA0,  ADC_IN1)
//   [1] = Ib  (PA1,  ADC_IN2)
//   [2] = Ic  (PB15, ADC2_IN15)
//
// REF fix ertekkel: config::kCurrentRefVoltage = 1.65V (nem merve)

namespace adc_dma {

// Inicializalja az ADC1-et DMA korkorosei uzemban.
// Hivd setup()-ban, a pinMode(INPUT_ANALOG) hiivasok utan.
// Visszateresi ertek: true ha sikeres; false eseten getRaw() 0-t ad vissza.
bool begin();

// Az adott csatorna legutobb konvertalt nyers ADC erteke (12 bit, 0..4095).
//   idx 0 = Ia, 1 = Ib, 2 = Ic, 3 = Ref
// Blokkolasmentes: a DMA a hatterben folyamatosan frissiti, a CPU nem var.
uint16_t getRaw(uint8_t idx);

}  // namespace adc_dma
