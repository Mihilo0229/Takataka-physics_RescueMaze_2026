/*******************************************************************************************/
/* NeoPixelのいろいろな動作関数                                                                            
/*処理：Neopixelを前後左右、赤黄緑に光らせる
/*更新者：清田侑希 2025/3/25
/*
/*******************************************************************************************/

void NeoPixel_Front_ON() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 0番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(1, pixels.Color(255, 0, 0)); // 1番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Front_OFF() {
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // 0番目のLEDを消灯
  pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // 1番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Right_ON() {
  pixels.setPixelColor(2, pixels.Color(255, 0, 0)); // 2番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // 3番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Right_OFF() {
  pixels.setPixelColor(2, pixels.Color(0, 0, 0)); // 2番目のLEDを消灯
  pixels.setPixelColor(3, pixels.Color(0, 0, 0)); // 3番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Rear_ON() {
  pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // 4番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(5, pixels.Color(255, 0, 0)); // 5番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Rear_OFF() {
  pixels.setPixelColor(4, pixels.Color(0, 0, 0)); // 4番目のLEDを消灯
  pixels.setPixelColor(5, pixels.Color(0, 0, 0)); // 5番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Left_ON() {
  pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // 6番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // 7番目のLEDを(R,G,B)=(255,0,0)⇔赤に点灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Left_OFF() {
  pixels.setPixelColor(6, pixels.Color(0, 0, 0)); // 6番目のLEDを消灯
  pixels.setPixelColor(7, pixels.Color(0, 0, 0)); // 7番目のLEDを消灯
  pixels.show(); // LEDに反映
}
void NeoPixel_Color(uint8_t r, uint8_t g, uint8_t b) {
  pixels.clear();
  pixels.fill(pixels.Color(r, g, b));
  pixels.show();
}