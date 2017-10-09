/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_hd44780.h"
#include "main.h"
/* Private HD44780 structure */
typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t Rows;
	uint8_t Cols;
	uint8_t currentX;
	uint8_t currentY;
} HD44780_Options_t;

/* Private functions */
//static void TM_HD44780_InitPins(void); Это нам не надо, так как делаем всё в cubeMX
static void TM_HD44780_Cmd(uint8_t cmd);
static void TM_HD44780_Cmd4bit(uint8_t cmd);
static void TM_HD44780_Data(uint8_t data);
static void TM_HD44780_CursorSet(uint8_t col, uint8_t row);

/// @brief Brief of СonvertRus
///
/// Функция замены двухбайтовых русских символов
/// однобайтовыми
CONVERTtoRUS ConvertRus(uint16_t rusSymb);

/* Private variable */
static HD44780_Options_t HD44780_Opts;

/* Pin definitions */
#define HD44780_RS_LOW              TM_GPIO_SetPinLow(HD44780_RS_PORT, HD44780_RS_PIN)
#define HD44780_RS_HIGH             TM_GPIO_SetPinHigh(HD44780_RS_PORT, HD44780_RS_PIN)
#define HD44780_E_LOW               TM_GPIO_SetPinLow(HD44780_E_PORT, HD44780_E_PIN)
#define HD44780_E_HIGH              TM_GPIO_SetPinHigh(HD44780_E_PORT, HD44780_E_PIN)

#define HD44780_E_BLINK             HD44780_E_HIGH; HD44780_Delay(20); HD44780_E_LOW; HD44780_Delay(20)
#define HD44780_Delay(x)            Delay(x)

/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

void TM_HD44780_Init(uint8_t cols, uint8_t rows) {
	/* Initialize delay */
	TM_DELAY_Init();

	/* Init pinout */
	//	TM_HD44780_InitPins();  Это нам не надо, так как делаем всё в cubeMX

	/* At least 40ms */
	//	HD44780_Delay(40000);

	/* Set LCD width and height */
	HD44780_Opts.Rows = rows;
	HD44780_Opts.Cols = cols;

	/* Set cursor pointer to beginning for LCD */
	HD44780_Opts.currentX = 0;
	HD44780_Opts.currentY = 0;

	HD44780_Opts.DisplayFunction = HD44780_4BITMODE | HD44780_5x8DOTS | HD44780_1LINE;
	if (rows > 1) {
		HD44780_Opts.DisplayFunction |= HD44780_2LINE;
	}

	/* Try to set 4bit mode */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(4500);

	/* Second try */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(4500);

	/* Third goo! */
	TM_HD44780_Cmd4bit(0x03);
	HD44780_Delay(4500);	

	/* Set 4-bit interface */
	TM_HD44780_Cmd4bit(0x02);
	HD44780_Delay(100);

	/* Set # lines, font size, etc. */
	TM_HD44780_Cmd(HD44780_FUNCTIONSET | HD44780_Opts.DisplayFunction);

	/* Turn the display on with no cursor or blinking default */
	HD44780_Opts.DisplayControl = HD44780_DISPLAYON;
	TM_HD44780_DisplayOn();

	/* Clear lcd */
	TM_HD44780_Clear();

	/* Default font directions */
	HD44780_Opts.DisplayMode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
	TM_HD44780_Cmd(HD44780_ENTRYMODESET | HD44780_Opts.DisplayMode);

	/* Delay */
	HD44780_Delay(4500);
}

void TM_HD44780_Clear(void) {
	TM_HD44780_Cmd(HD44780_CLEARDISPLAY);
	HD44780_Delay(3000);
}


void TM_HD44780_Puts(uint8_t x, uint8_t y, uint8_t *str, uint8_t len)
{
	uint16_t in2byte;
	uint8_t numOfsymbol = 0;
	uint8_t *strScore = str;
	uint8_t length = len;

	// Пробегаемся по тексту и считаем количество символов.
	// Это требуется делать, так как мы не знаем количество символов,
	// а знаем только количество байт. Кириллица кодируется двумя байтами,
	// остальные символы - одним.
	while(length != 0)
	{
		// Если не встретили подобные значения(с этого начинается двухбайтовый
		// символ кириллицы), то к счётчику символов прибавляем 1, а если встретили,
		// то прибавляем 2.
		if((*strScore == 0xD0) || (*strScore == 0xD1))
		{
			strScore += 2;
			length -= 2;
		}
		else
		{

			strScore ++;
			length --;
		}
		numOfsymbol ++;
	}

	// Определяем номер ячейки по горизонтали, чтобы
	// текст был посередине
	x = (LENGTH_OF_LINE_LCD/2) - (numOfsymbol/2);

	// Если текст был не кратен двум, то свдигем
	// ячейку на единицу влево
	if(x % 2) x--;

	TM_HD44780_CursorSet(x, y);

	while(len != 0)
	{
		/*Так я пометил место изменения. Если убрать комменты и удалить строчки, то будет норм*/
		//		// 1 - rus
		//		// 0 - eng
		//		if(language)
		//		{
		// Если встретили подобные символы в массиве, то делай следующее:
		if (*str == '\n')														/**/
		{                                         								/**/
			// Прибавляем единицу к смещению курсора по вертикали				/**/
			HD44780_Opts.currentY++;											/**/
			// Устанавливаем курсор на новое место								/**/
			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY); /**/
			len--;
			str++;
		} else if (*str == '\r')												/**/
		{																		/**/
			// Устанавливаем курсор в начало строки								/**/
			TM_HD44780_CursorSet(0, HD44780_Opts.currentY);						/**/
			len--;
			str++;
		}																		/**/
		else																	/**/
		{																		/**/
			// Если смещение курсора больше, чем длина строки дисплея,
			// то скорллим строчку
			if (HD44780_Opts.currentX >= HD44780_Opts.Cols)
			{
				// Ждём 500 тактов....зачем?
				HAL_Delay(500);
				// Скроллим фразу, то есть двигаем её влево, а справа
				// у нас появляется свободная ячейка
				TM_HD44780_ScrollLeft();
				// Отбираем единицу у смещения курсора по горизонтали
				HD44780_Opts.currentX--;
			}
			// Смотрим на первый байт массива. Если он похож на старший
			// байт двухбайтового слова, которым кодируется кириллица, то
			// закидываем его в двухбайтовую переменную, двигаем на место
			// старшего байта, а затем берем следующий однобайтовый элемент
			// массива и лепим его в качестве младшего байта нашей
			// двухбайтовой переменной
			if((*str == 0xD0) || (*str == 0xD1))
			{
				// Обнуляем переменную
				in2byte = 0;
				// Делаем тяп-ляп
				str++;
				in2byte = *(str);
				in2byte = in2byte << 8;
				str--;
				in2byte = in2byte | *(str);
				//in2byte = ((in2byte & *str) << 8) | *(str++);
				// двигаем счётчик
				str += 2;
				len -= 2;
				// Отправляем наш двухбайтовый массив функции ConvertRus()
				// , которая вернёт нам уже нужный байт, понятный для драйвера
				// дисплея. И сразу же отправляем его дисплею
				TM_HD44780_Data(ConvertRus(in2byte));
				numOfsymbol++;
			}
			else
			{
				// Если это не кириллица, то просто отправляем байт дисплею.
				TM_HD44780_Data(*str);
				numOfsymbol++;
				// Двигаем счётчик на единицу
				str++;
				len--;
			}
			// Двигаем курсор вправо
			HD44780_Opts.currentX++;

			//		}
			//		else
			//		{
			//		if (HD44780_Opts.currentX >= HD44780_Opts.Cols)
			//		{
			//			HAL_Delay(500);
			//			TM_HD44780_ScrollLeft();
			//			HD44780_Opts.currentX--;
			//		}
			//		if (*str == '\n') {
			//			HD44780_Opts.currentY++;
			//			TM_HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
			//		} else if (*str == '\r') {
			//			TM_HD44780_CursorSet(0, HD44780_Opts.currentY);
			//		} else {
			//			TM_HD44780_Data(*str);
			//			HD44780_Opts.currentX++;
			//		}
			//		str++;
			//		}
			/**/
		}
	}
}

void TM_HD44780_DisplayOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_DisplayOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_BlinkOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_CursorOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
	TM_HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void TM_HD44780_ScrollLeft(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void TM_HD44780_ScrollRight(void) {
	TM_HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void TM_HD44780_CreateChar(uint8_t location, uint8_t *data) {
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	TM_HD44780_Cmd(HD44780_SETCGRAMADDR | (location << 3));

	for (i = 0; i < 8; i++) {
		TM_HD44780_Data(data[i]);
	}
}

void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location) {
	TM_HD44780_CursorSet(x, y);
	TM_HD44780_Data(location);
}

/* Private functions */
static void TM_HD44780_Cmd(uint8_t cmd) {
	/* Command mode */
	HD44780_RS_LOW;

	/* High nibble */
	TM_HD44780_Cmd4bit(cmd >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(cmd & 0x0F);
}

static void TM_HD44780_Data(uint8_t data) {
	/* Data mode */
	HD44780_RS_HIGH;

	/* High nibble */
	TM_HD44780_Cmd4bit(data >> 4);
	/* Low nibble */
	TM_HD44780_Cmd4bit(data & 0x0F);
}

static void TM_HD44780_Cmd4bit(uint8_t cmd) {
	/* Set output port */
	TM_GPIO_SetPinValue(HD44780_D7_PORT, HD44780_D7_PIN, (cmd & 0x08));
	TM_GPIO_SetPinValue(HD44780_D6_PORT, HD44780_D6_PIN, (cmd & 0x04));
	TM_GPIO_SetPinValue(HD44780_D5_PORT, HD44780_D5_PIN, (cmd & 0x02));
	TM_GPIO_SetPinValue(HD44780_D4_PORT, HD44780_D4_PIN, (cmd & 0x01));
	HD44780_E_BLINK;
}

static void TM_HD44780_CursorSet(uint8_t col, uint8_t row) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

	/* Go to beginning */
	if (row >= HD44780_Opts.Rows) {
		row = 0;
	}

	/* Set current column and row */
	HD44780_Opts.currentX = col;
	HD44780_Opts.currentY = row;

	/* Set location address */
	TM_HD44780_Cmd(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}

// Инициализация пинов нам не нужна, так как мы всё делаем в CubeMX
//static void TM_HD44780_InitPins(void) {
//	/* Init all pins */
//	TM_GPIO_Init(HD44780_RS_PORT, HD44780_RS_PIN, TM_GPIO_Mode_OUT_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_E_PORT, HD44780_E_PIN, TM_GPIO_Mode_OUT_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D4_PORT, HD44780_D4_PIN, TM_GPIO_Mode_OUT_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D5_PORT, HD44780_D5_PIN, TM_GPIO_Mode_OUT_PP,  TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D6_PORT, HD44780_D6_PIN, TM_GPIO_Mode_OUT_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//	TM_GPIO_Init(HD44780_D7_PORT, HD44780_D7_PIN, TM_GPIO_Mode_OUT_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
//
//	/* Set pins low */
//	TM_GPIO_SetPinLow(HD44780_RS_PORT, HD44780_RS_PIN);
//	TM_GPIO_SetPinLow(HD44780_E_PORT, HD44780_E_PIN);
//	TM_GPIO_SetPinLow(HD44780_D4_PORT, HD44780_D4_PIN);
//	TM_GPIO_SetPinLow(HD44780_D5_PORT, HD44780_D5_PIN);
//	TM_GPIO_SetPinLow(HD44780_D6_PORT, HD44780_D6_PIN);
//	TM_GPIO_SetPinLow(HD44780_D7_PORT, HD44780_D7_PIN);
//}

CONVERTtoRUS ConvertRus(uint16_t rusSymb)
{
	switch(rusSymb)
	{
	case 0x90D0:   //А
		return RUS00;
		break;
	case 0x91D0:   //Б
		return RUS01;
		break;
	case 0x92D0:   //В
		return RUS02;
		break;
	case 0x93D0:   //Г
		return RUS03;
		break;
	case 0x94D0:   //Д
		return RUS04;
		break;
	case 0x95D0:   //Е
		return RUS05;
		break;
	case 0x81D0:   //Ё
		return RUS06;
		break;
	case 0x96D0:   //Ж
		return RUS07;
		break;
	case 0x97D0:   //З
		return RUS08;
		break;
	case 0x98D0:   //И
		return RUS09;
		break;
	case 0x99D0:   //Й
		return RUS0A;
		break;
	case 0x9AD0:   //К
		return RUS0B;
		break;
	case 0x9BD0:   //Л
		return RUS0C;
		break;
	case 0x9CD0:   //М
		return RUS0D;
		break;
	case 0x9DD0:   //Н
		return RUS0E;
		break;
	case 0x9ED0:   //О
		return RUS0F;
		break;
	case 0x9FD0:   //П
		return RUS10;
		break;
	case 0xA0D0:   //Р
		return RUS11;
		break;
	case 0xA1D0:   //С
		return RUS12;
		break;
	case 0xA2D0:   //Т
		return RUS13;
		break;
	case 0xA3D0:   //У
		return RUS14;
		break;
	case 0xA4D0:   //Ф
		return RUS15;
		break;
	case 0xA5D0:   //Х
		return RUS16;
		break;
	case 0xA6D0:   //Ц
		return RUS17;
		break;
	case 0xA7D0:   //Ч
		return RUS18;
		break;
	case 0xA8D0:   //Ш
		return RUS19;
		break;
	case 0xA9D0:   //Щ
		return RUS1A;
		break;
	case 0xAAD0:   //Ъ
		return RUS1B;
		break;
	case 0xABD0:   //Ы
		return RUS1C;
		break;
	case 0xACD0:   //Ь
		return RUS1D;
		break;
	case 0xADD0:   //Э
		return RUS1E;
		break;
	case 0xAED0:   //Ю
		return RUS1F;
		break;
	case 0xAFD0:   //Я
		return RUS20;
		break;
	case 0xB0D0:   //а
		return RUS21;
		break;
	case 0xB1D0:   //б
		return RUS22;
		break;
	case 0xB2D0:   //в
		return RUS23;
		break;
	case 0xB3D0:   //г
		return RUS24;
		break;
	case 0xB4D0:   //д
		return RUS25;
		break;
	case 0xB5D0:   //е
		return RUS26;
		break;
	case 0x91D1:   //ё
		return RUS27;
		break;
	case 0xB6D0:   //ж
		return RUS28;
		break;
	case 0xB7D0:   //з
		return RUS29;
		break;
	case 0xB8D0:   //и
		return RUS2A;
		break;
	case 0xB9D0:   //й
		return RUS2B;
		break;
	case 0xBAD0:   //к
		return RUS2C;
		break;
	case 0xBBD0:   //л
		return RUS2D;
		break;
	case 0xBCD0:   //м
		return RUS2E;
		break;
	case 0xBDD0:   //н
		return RUS2F;
		break;
	case 0xBED0:   //о
		return RUS30;
		break;
	case 0xBFD0:   //п
		return RUS31;
		break;
	case 0x80D1:   //р
		return RUS32;
		break;
	case 0x81D1:   //с
		return RUS33;
		break;
	case 0x82D1:   //т
		return RUS34;
		break;
	case 0x83D1:   //у
		return RUS35;
		break;
	case 0x84D1:   //ф
		return RUS36;
		break;
	case 0x85D1:   //х
		return RUS37;
		break;
	case 0x86D1:   //ц
		return RUS38;
		break;
	case 0x87D1:   //ч
		return RUS39;
		break;
	case 0x88D1:   //ш
		return RUS3A;
		break;
	case 0x89D1:   //щ
		return RUS3B;
		break;
	case 0x8AD1:   //ъ
		return RUS3C;
		break;
	case 0x8BD1:   //ы
		return RUS3D;
		break;
	case 0x8CD1:   //ь
		return RUS3E;
		break;
	case 0x8DD1:   //э
		return RUS3F;
		break;
	case 0x8ED1:   //ю
		return RUS40;
		break;
	case 0x8FD1:   //я
		return RUS41;
		break;
	default:
		return RUSerror;
		break;
	}
	return RUSerror;
}
