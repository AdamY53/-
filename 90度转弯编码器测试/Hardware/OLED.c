#include "stm32f10x.h"
#include "OLED.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

uint8_t OLED_DisplayBuf[8][128];

static void OLED_Wait(void)
{
	uint8_t i;
	for (i = 0; i < 12; i++);
}

void OLED_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(OLED_GPIO_PORT, OLED_SCL_PIN, (BitAction)BitValue);
	OLED_Wait();
}

void OLED_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(OLED_GPIO_PORT, OLED_SDA_PIN, (BitAction)BitValue);
	OLED_Wait();
}

void OLED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t i;
	
	RCC_APB2PeriphClockCmd(OLED_GPIO_CLK, ENABLE);
	for (i = 0; i < 10000; i++);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStructure);
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA((Byte & (0x80 >> i)) ? 1 : 0);
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);
	OLED_W_SCL(0);
}

void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
}

void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
	uint8_t i;
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);
	OLED_I2C_SendByte(0x40);
	for (i = 0; i < Count; i++)
	{
		OLED_I2C_SendByte(Data[i]);
	}
	OLED_I2C_Stop();
}

void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Page);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Update(void)
{
	uint8_t j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		OLED_WriteData(OLED_DisplayBuf[j], 128);
	}
}

void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t PageStart, PageEnd, Page;
	if (X < 0) {X = 0;}
	if (Y < 0) {Y = 0;}
	if (X > 127 || Y > 63) {return;}
	if (X + Width > 128) {Width = 128 - X;}
	PageStart = Y / 8;
	PageEnd = (Y + Height - 1) / 8;
	if (PageEnd > 7) {PageEnd = 7;}
	for (Page = PageStart; Page <= PageEnd; Page++)
	{
		OLED_SetCursor((uint8_t)Page, (uint8_t)X);
		OLED_WriteData(&OLED_DisplayBuf[Page][X], Width);
	}
}

void OLED_Clear(void)
{
	memset(OLED_DisplayBuf, 0, sizeof(OLED_DisplayBuf));
}

void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t x, y;
	for (y = Y; y < Y + Height; y++)
	{
		for (x = X; x < X + Width; x++)
		{
			if (x >= 0 && x < 128 && y >= 0 && y < 64)
			{
				OLED_DisplayBuf[y / 8][x] &= ~(0x01 << (y % 8));
			}
		}
	}
}

void OLED_Reverse(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		for (i = 0; i < 128; i++)
		{
			OLED_DisplayBuf[j][i] ^= 0xFF;
		}
	}
}

void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
	int16_t x, y;
	for (y = Y; y < Y + Height; y++)
	{
		for (x = X; x < X + Width; x++)
		{
			if (x >= 0 && x < 128 && y >= 0 && y < 64)
			{
				OLED_DisplayBuf[y / 8][x] ^= (0x01 << (y % 8));
			}
		}
	}
}

void OLED_DrawPoint(int16_t X, int16_t Y)
{
	if (X >= 0 && X < 128 && Y >= 0 && Y < 64)
	{
		OLED_DisplayBuf[Y / 8][X] |= (0x01 << (Y % 8));
	}
}

uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
	if (X >= 0 && X < 128 && Y >= 0 && Y < 64)
	{
		return (OLED_DisplayBuf[Y / 8][X] & (0x01 << (Y % 8))) ? 1 : 0;
	}
	return 0;
}

void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
	uint8_t i, j;
	for (j = 0; j < Height; j++)
	{
		for (i = 0; i < Width; i++)
		{
			if (Image[(j / 8) * Width + i] & (0x01 << (j % 8)))
			{
				OLED_DrawPoint(X + i, Y + j);
			}
		}
	}
}

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
	uint8_t Index;
	if (Char < ' ' || Char > '~') {Char = ' ';}
	Index = (uint8_t)(Char - ' ');
	if (FontSize == OLED_8X16)
	{
		OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Index]);
	}
	else
	{
		OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Index]);
	}
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
	uint8_t i = 0;
	uint8_t Step = (FontSize == OLED_8X16) ? 8 : 6;
	while (String[i] != '\0')
	{
		OLED_ShowChar(X + i * Step, Y, String[i], FontSize);
		i++;
	}
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--) {Result *= X;}
	return Result;
}

void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	uint8_t Step = (FontSize == OLED_8X16) ? 8 : 6;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(X + i * Step, Y, (Number / OLED_Pow(10, Length - i - 1)) % 10 + '0', FontSize);
	}
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t Step = (FontSize == OLED_8X16) ? 8 : 6;
	uint32_t AbsNum;
	if (Number >= 0)
	{
		OLED_ShowChar(X, Y, '+', FontSize);
		AbsNum = Number;
	}
	else
	{
		OLED_ShowChar(X, Y, '-', FontSize);
		AbsNum = (uint32_t)(-Number);
	}
	OLED_ShowNum(X + Step, Y, AbsNum, Length, FontSize);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i, Single;
	uint8_t Step = (FontSize == OLED_8X16) ? 8 : 6;
	for (i = 0; i < Length; i++)
	{
		Single = (Number >> (4 * (Length - i - 1))) & 0x0F;
		OLED_ShowChar(X + i * Step, Y, (Single < 10) ? (Single + '0') : (Single - 10 + 'A'), FontSize);
	}
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	uint8_t Step = (FontSize == OLED_8X16) ? 8 : 6;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(X + i * Step, Y, ((Number >> (Length - i - 1)) & 0x01) + '0', FontSize);
	}
}

void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
	char Buf[24];
	char Fmt[8];
	sprintf(Fmt, "%%.%df", FraLength);
	sprintf(Buf, Fmt, Number);
	OLED_ShowString(X, Y, Buf, FontSize);
}

void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
	char Buf[40];
	va_list arg;
	va_start(arg, format);
	vsprintf(Buf, format, arg);
	va_end(arg);
	OLED_ShowString(X, Y, Buf, FontSize);
}

void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
	int16_t dx = (X1 > X0) ? (X1 - X0) : (X0 - X1);
	int16_t sx = (X0 < X1) ? 1 : -1;
	int16_t dy = (Y1 > Y0) ? (Y0 - Y1) : (Y1 - Y0);
	int16_t sy = (Y0 < Y1) ? 1 : -1;
	int16_t err = dx + dy;
	int16_t e2;
	while (1)
	{
		OLED_DrawPoint(X0, Y0);
		if (X0 == X1 && Y0 == Y1) {break;}
		e2 = 2 * err;
		if (e2 >= dy) {err += dy; X0 += sx;}
		if (e2 <= dx) {err += dx; Y0 += sy;}
	}
}

void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
	uint8_t i, j;
	if (IsFilled)
	{
		for (j = 0; j < Height; j++)
		{
			for (i = 0; i < Width; i++) {OLED_DrawPoint(X + i, Y + j);}
		}
	}
	else
	{
		OLED_DrawLine(X, Y, X + Width - 1, Y);
		OLED_DrawLine(X, Y + Height - 1, X + Width - 1, Y + Height - 1);
		OLED_DrawLine(X, Y, X, Y + Height - 1);
		OLED_DrawLine(X + Width - 1, Y, X + Width - 1, Y + Height - 1);
	}
}

void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
	(void)IsFilled;
	OLED_DrawLine(X0, Y0, X1, Y1);
	OLED_DrawLine(X1, Y1, X2, Y2);
	OLED_DrawLine(X2, Y2, X0, Y0);
}

void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
	int16_t x, y;
	for (y = -Radius; y <= Radius; y++)
	{
		for (x = -Radius; x <= Radius; x++)
		{
			int16_t d = x * x + y * y;
			int16_t r = Radius * Radius;
			if ((IsFilled && d <= r) || (!IsFilled && d >= r - Radius && d <= r + Radius))
			{
				OLED_DrawPoint(X + x, Y + y);
			}
		}
	}
}

void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
	int16_t x, y;
	int32_t aa = (int32_t)A * A;
	int32_t bb = (int32_t)B * B;
	int32_t rr = aa * bb;
	for (y = -B; y <= B; y++)
	{
		for (x = -A; x <= A; x++)
		{
			int32_t v = (int32_t)x * x * bb + (int32_t)y * y * aa;
			if ((IsFilled && v <= rr) || (!IsFilled && v >= rr - aa - bb && v <= rr + aa + bb))
			{
				OLED_DrawPoint(X + x, Y + y);
			}
		}
	}
}

void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
	(void)StartAngle;
	(void)EndAngle;
	OLED_DrawCircle(X, Y, Radius, IsFilled);
}

void OLED_Init(void)
{
	OLED_GPIO_Init();
	OLED_WriteCommand(0xAE);
	OLED_WriteCommand(0xD5);
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);
	OLED_WriteCommand(0xA1);
	OLED_WriteCommand(0xC8);
	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);
	OLED_WriteCommand(0xA6);
	OLED_WriteCommand(0x8D);
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0xAF);
	OLED_Clear();
	OLED_Update();
}
