#include "oled.h"
#include "imu.h"
#include "odrive.h"
extern imu_t imu;
extern OdirveTypeDef odrive;
char imu_rol[5];
char enc1[5];
char enc2[5];
void oled_init(void) {
	ssd1306_Init();
  ssd1306_FlipScreenVertically();
  ssd1306_Clear();
  ssd1306_SetColor(White);
	ssd1306_SetCursor(20,10);
	ssd1306_WriteString("IMU:", Font_7x10);
	ssd1306_SetCursor(20,25);
	ssd1306_WriteString("ENC1:", Font_7x10);
	ssd1306_SetCursor(20,40);
	ssd1306_WriteString("ENC2:", Font_7x10);
	ssd1306_UpdateScreen();
}

void oled_flush(void) {
	sprintf(imu_rol,"%.2f",imu.rol);
	ssd1306_SetCursor(50,10);
	ssd1306_WriteString(imu_rol, Font_7x10);
	sprintf(enc1,"%.2f",odrive.now_speed0);
	ssd1306_SetCursor(55,25);
	ssd1306_WriteString(enc1, Font_7x10);
	sprintf(enc2,"%.2f",odrive.now_speed1);
	ssd1306_SetCursor(55,40);
	ssd1306_WriteString(enc2, Font_7x10);
	ssd1306_UpdateScreen();
}
