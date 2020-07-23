#include <stdio.h>
#include "stdlib.h"
#include "Core.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)

#include "MyButton.h"
//#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"
#include "mapCar.h"

#define LED_PIN         13
#define LED_PORT        GPIOC
#define LED_PIN_SPEED   GPIO_Speed_2MHz

#define LED_ON()       LED_PORT->BRR  = _BV(LED_PIN)
#define LED_OFF()      LED_PORT->BSRR = _BV(LED_PIN)
#define LED_TOGGLE()    BITMASK_TGL(LED_PORT->ODR, _BV(LED_PIN))

/*===================================Pin Define===========================================*/
#define   CLK_PIN   8
#define   DIN_PIN   7
#define   DC_PIN    6
#define   CS_PIN    5
#define   RST_PIN   4

#define  EAST_PIN   9
#define  WEST_PIN   11
#define SOUTH_PIN   8
#define NORTH_PIN   10
#define    OK_PIN   12
#define PAUSE_PIN   10
#define  QUIT_PIN   13

MyButton  EAST(GPIOA, EAST_PIN);
MyButton  WEST(GPIOB, WEST_PIN);
MyButton SOUTH(GPIOA,SOUTH_PIN);
MyButton NORTH(GPIOA,NORTH_PIN);
MyButton    OK(GPIOB,   OK_PIN);
MyButton PAUSE(GPIOB,   PAUSE_PIN);
MyButton  QUIT(GPIOB,   QUIT_PIN);
/*========================================================================================*/

uint8_t MODE = 1;


Adafruit_PCD8544 lcd(GPIOB,CLK_PIN,DIN_PIN,DC_PIN,CS_PIN,RST_PIN);

class Brick : public Adafruit_GFX {
 public:
     Brick(uint8_t a = 40, uint8_t b = 40);
     void setStatus(uint8_t state);
     uint8_t getStatus();
     void init(uint8_t x, uint8_t y);
 private:
     uint8_t x;
     uint8_t y;
     uint8_t status;
};

Brick::Brick(uint8_t a, uint8_t b):
    Adafruit_GFX(84, 48),
    x(a),
    y(b),
    status(0)
{
    setRotation(3);
}

void Brick::setStatus(uint8_t state) {
    this->status = state;
    if ( state == 1 )
        fillRect(x,y,3,3,BLACK);
    else if ( state == 2 ) {
        drawRect(x,y,3,3,BLACK);
    } else if ( state == 0 ){
        fillRect(x,y,3,3,WHITE);
    }
}

uint8_t Brick::getStatus() {
    return this->status;
}

void Brick::init(uint8_t x, uint8_t y) {
    this->x      = x;
    this->y      = y;
    this->status = 0;
}


void gameBackGround(){
/*-----------Back ground game----------------*/
    lcd.clearDisplay();   
    lcd.drawTriangle(0,0,47,0,25,3, BLACK);
    lcd.fillTriangle(0,0,47,0,25,3, BLACK);
    lcd.drawLine(0,22,31,22,BLACK);
    lcd.drawLine(31,22,31,83,BLACK);

    lcd.drawLine(34,71,46,71,BLACK);
    lcd.setFont(PCD8544_FONT3X5);
    lcd.setCursor(0, 6);
    lcd.print("score");
    lcd.setCursor(0, 15);
    lcd.print("Hi-score");
    lcd.setCursor(33, 55);
    lcd.print("Goal");
    lcd.setCursor(33, 29);
    lcd.print("Lv");


/*-----------End back ground----------------*/
}


/*-----------Menu----------------*/
void menu() {
    uint8_t mod = 2;

    lcd.clearDisplay();   
    lcd.drawTriangle(0,0,47,0,25,5, BLACK);
    lcd.fillTriangle(0,0,47,0,25,5, BLACK);
    lcd.drawLine(0,19,47,19,BLACK);

    lcd.drawTriangle(0,83,47,83,25,76, BLACK);
    lcd.fillTriangle(0,83,47,83,25,76, BLACK);
    lcd.drawLine(0,73,47,73,BLACK);

    lcd.setFont(PCD8544_FONT5X7);
    lcd.setCursor(12, 9);
    lcd.print("MENU");


    lcd.setCursor(3, 23);
    lcd.print("Car");
    lcd.setCursor(3, 33);
    lcd.print("Snake");
    lcd.setCursor(3, 43);
    lcd.print("Tetris");
    lcd.setCursor(3, 53);
    lcd.print("Pong");
    lcd.setCursor(3, 63);
    lcd.print("Setting");


/*-----------End menu----------------*/ 


    lcd.fillRectInvert(0, mod*10+1, 48, 11);
    lcd.display();
    while(1){
        if (NORTH.onPress() || NORTH.isHold()) {
            lcd.fillRectInvert(0, mod*10+1, 48, 11);
            mod--;
            if (mod == 1) mod = 6;
            lcd.fillRectInvert(0, mod*10+1, 48, 11);
            lcd.display();
        } else if (SOUTH.onPress() || SOUTH.isHold()) {
            lcd.fillRectInvert(0, mod*10+1, 48, 11);
            mod++;
            if (mod == 7) mod = 2;
            lcd.fillRectInvert(0, mod*10+1, 48, 11);
            lcd.display();
        } else if (OK.onPress()) {
            MODE = mod;
            break;
        }
        LED_TOGGLE();
        delay_ms(50);
    }
}

void game1() {
    uint32_t Address = (uint32_t)0x0801FC00;
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
    volatile TestStatus MemoryProgramStatus = PASSED;

    gameBackGround();
    lcd.setCursor(37, 75);// Biến này set cố định. Nếu goal == 11 -> set goal = 1
    lcd.print(10);
/*--------------------*/
    Brick brick[20][10];
    uint8_t x = 0, y = 24;
    for (uint8_t i = 0; i < 20; i++) {
        for (uint8_t j = 0; j < 10; j++) {
            brick[i][j].init(x,y);
            x+=3;
        }
        x  = 0;
        y += 3;
    }

    uint8_t j = 3;
    for (uint8_t i = 0; i < 20; i++) {
        if (i == j){
            brick[i][0].setStatus(0);
            brick[i][9].setStatus(0);
            j+=4;
        } else {
            brick[i][0].setStatus(1);
            brick[i][9].setStatus(1);
        }
    }


    //Bảng ma trận BRICK
    //Bảng hiện thị level
    lcd.fillRect(33,38,15,12,BLACK);
    lcd.fillRect(33,38,15,12,BLACK);


    lcd.display(); 

    // uint8_t u8beginPointStatus = 0, u8nextPointStatus ;
    uint8_t index = 0;
    uint8_t indexCar = 0;
    uint8_t shift = 0;
    uint8_t dir = 0;
    uint8_t time_delay = 0;
    uint8_t time_delay2 = 100;
    uint8_t distanceEnemy = 10;

    uint8_t Goal = 0;
    uint16_t Score = 0;
    uint16_t hiScore = (*(__IO uint32_t*) Address);
    uint8_t Level = 1;


    lcd.fillRect(33,6,15,5,WHITE);
    lcd.fillRect(37,63,7,5,WHITE);
    lcd.fillRect(41,29,7,5,WHITE);
    lcd.setCursor(33, 6);//Score
    lcd.print(Score);
    lcd.setCursor(37, 63);// Biến goal chạy theo từng bước ghi điểm
    lcd.print(Goal);
    lcd.setCursor(41, 29);
    lcd.print(Level);
    lcd.setCursor(33, 15);//Hi-score
    lcd.print(hiScore);

    bool pause = 0;
    while(1){
        if(PAUSE.onPress() || PAUSE.isHold()){
            pause = ! pause;
        }

        if(pause == 0) {
            uint8_t u8beginPointStatus[10], u8nextPointStatus[10] ;
    /*-------------------------Dịch 2 bên viền game-----------------------------------*/
            u8beginPointStatus[0] = brick[19][0].getStatus();
            for (int8_t Point = 19; Point >= 0; Point--) {
                if (Point == 0) {
                    brick[Point][0].setStatus(u8beginPointStatus[0]);
                    brick[Point][9].setStatus(u8beginPointStatus[0]);
                } else {
                    u8nextPointStatus[0] =  brick[Point-1][0].getStatus();
                    brick[Point][0].setStatus(u8nextPointStatus[0]);
                    brick[Point][9].setStatus(u8nextPointStatus[0]);
                }
            }


    /*-----------------------Dịch 2 làn xe----------------------------------*/
            /*======Khởi tạo 6 cột đầu tiên==========*/
            for (int8_t i = 2; i < 8; i++) {
                u8beginPointStatus[i] = 0;
            }
            //lcd.print(mapCar[0][0]);
            uint8_t statusLine1 = !!((mapCar[indexCar][0]>>shift)&0x01);
            uint8_t statusLine2 = !!((mapCar[indexCar][1]>>shift)&0x01);
            if(index == 3) {
                u8beginPointStatus[3] = statusLine1;
                u8beginPointStatus[6] = statusLine2;
            } else if(index == 2) {
                for (int8_t i = 2; i < 5; i++) {
                    u8beginPointStatus[i] = statusLine1;
                }
                for (int8_t i = 5; i < 8; i++) {
                    u8beginPointStatus[i] = statusLine2;
                }
            } else if(index == 1) {
                u8beginPointStatus[3] = statusLine1;
                u8beginPointStatus[6] = statusLine2;
            } else if(index == 0) {
                u8beginPointStatus[2] = statusLine1;
                u8beginPointStatus[4] = statusLine1;
                u8beginPointStatus[5] = statusLine2;
                u8beginPointStatus[7] = statusLine2;
            }

            for (int8_t Point = 19; Point >= 0; Point--) {
                for (int8_t i = 2; i < 8; i++) {
                    if (Point == 0) {
                        brick[Point][i].setStatus(u8beginPointStatus[i]);
                    } else {
                        u8nextPointStatus[i] =  brick[Point-1][i].getStatus();
                        brick[Point][i].setStatus(u8nextPointStatus[i]);
                    }
                }
            }

            if(index == distanceEnemy){
                index = 0;
                if(shift == (NUMBER_OF_BIT-0)){
                    shift = 0;
                    if(indexCar == (NUMBER_OF_MAP-1)) {
                        indexCar = 0;
                    } else {
                        indexCar = indexCar + 1;
                    }
                } else {
                    shift = shift +1;
                }
            } else {
                index = index + 1;
            }


            uint8_t end = 0;
            if(WEST.onPress() || WEST.isHold()){
                for(uint8_t i = 16; i < 20; i++) {
                    for(uint8_t j = 2+dir; j < 5+dir; j++) {
                        brick[i][j].setStatus(0);
                    }
                }
                dir = 0;
                for(uint8_t i = 16; i < 20; i++) {
                    for(uint8_t j = 2+dir; j < 5+dir; j++) {
                        if(end < brick[i][j].getStatus())
                            end = brick[i][j].getStatus();
                    }
                }
            }
            if(EAST.onPress() || EAST.isHold()){
                for(uint8_t i = 16; i < 20; i++) {
                    for(uint8_t j = 2+dir; j < 5+dir; j++) {
                        brick[i][j].setStatus(0);
                    }
                }
                dir = 3;
                for(uint8_t i = 16; i < 20; i++) {
                    for(uint8_t j = 2+dir; j < 5+dir; j++) {
                        if(end < brick[i][j].getStatus())
                            end = brick[i][j].getStatus();
                    }
                }
            }
            if(OK.onPress() || OK.isHold()){
                time_delay  = 20;
            } else {
                time_delay = time_delay2;
            }

            if(brick[16][3+dir].getStatus() || end) {
                lcd.display();
                brick[16][2+dir].setStatus(0);
                brick[16][3+dir].setStatus(1);
                brick[16][4+dir].setStatus(0);
                brick[17][2+dir].setStatus(1);
                brick[17][3+dir].setStatus(0);
                brick[17][4+dir].setStatus(1);
                brick[18][2+dir].setStatus(1);
                brick[18][3+dir].setStatus(0);
                brick[18][4+dir].setStatus(1);
                brick[19][2+dir].setStatus(0);
                brick[19][3+dir].setStatus(1);
                brick[19][4+dir].setStatus(0);
                while(1){
                    lcd.fillRectInvert(6+3*dir, 72, 9, 12);
                    delay_ms(100);
                    lcd.display();
                    if(QUIT.onPress()){
                        for(uint8_t i = 0; i < 20; i++) {
                            for(uint8_t j = 2; j < 8; j++) {
                                brick[i][j].setStatus(0);
                            }
                        }
                        Goal = 1;
                        Level = 1;
                        Score = 0;
                        shift = NUMBER_OF_BIT;
                        index = 0;
                        lcd.fillRect(33,6,15,5,WHITE);
                        lcd.fillRect(37,63,7,5,WHITE);
                        lcd.fillRect(41,29,7,5,WHITE);

                        lcd.setCursor(33, 6);//Score
                        lcd.print(Score);
                        lcd.setCursor(37, 63);
                        lcd.print(Goal);
                        lcd.setCursor(41, 29);
                        lcd.print(Level);
                        break;
                    }
                }
            } else {
                brick[16][2+dir].setStatus(0);
                brick[16][3+dir].setStatus(1);
                brick[16][4+dir].setStatus(0);
                brick[17][2+dir].setStatus(1);
                brick[17][3+dir].setStatus(1);
                brick[17][4+dir].setStatus(1);
                brick[18][2+dir].setStatus(0);
                brick[18][3+dir].setStatus(1);
                brick[18][4+dir].setStatus(0);
                brick[19][2+dir].setStatus(1);
                brick[19][3+dir].setStatus(0);
                brick[19][4+dir].setStatus(1);
            }

            uint8_t pass = 1;
            for(uint8_t j = 2; j < 8; j++) {
                if(brick[17][j].getStatus()){
                    pass ++;
                }
            }

            if(pass == 6){
                lcd.fillRect(33,6,15,5,WHITE);
                lcd.fillRect(37,63,7,5,WHITE);
                lcd.fillRect(41,29,7,5,WHITE);
                Goal++;
                if(Goal == 11){
                    Goal = 1;
                    Score++;
                    if(!(Score%5)){
                        Level++;
                        if(time_delay2 > 10){
                            time_delay2 = time_delay2 - 20;
                            distanceEnemy --;
                            if(distanceEnemy < 4) distanceEnemy = 4;
                        }
                    }
                }
                if(Score>hiScore) {
                    hiScore = Score;
                    /* Porgram FLASH Bank1 ********************************************************/
                    /* Unlock the Flash Bank1 Program Erase controller */
                    FLASH_UnlockBank1();
                    //  /* Define the number of page to be erased */
                    uint32_t NbrOfPage = 1;
                    /* Clear All pending flags */
                    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
                    ////  /* Erase the FLASH pages */
                    for(uint32_t EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++) {
                    FLASHStatus = FLASH_ErasePage(Address + (FLASH_PAGE_SIZE * EraseCounter));
                    }
                    /* Program Flash Bank1 */
                    FLASHStatus = FLASH_ProgramWord(Address, hiScore);
                    FLASH_LockBank1();
                }
                lcd.setCursor(33, 6);
                lcd.print(Score);
                lcd.setCursor(37, 63);
                lcd.print(Goal);
                lcd.setCursor(41, 29);
                lcd.print(Level);
                lcd.setCursor(33, 15);
                lcd.print(hiScore);
            }



            LED_TOGGLE();
            lcd.display(); 
            delay_ms(time_delay);

        } else if(QUIT.onPress() || QUIT.isHold()){
            MODE = 1;
            break;
        } else {
            LED_TOGGLE();
            delay_ms(200);
        }
    }
}

void game2() {
    gameBackGround();
    while(1){
    }
}

void game3() {
    gameBackGround();
    while(1){
    }
}

void setting() {
    while(1){
    }
}
int main(void) {

   /// Setup
    // Core_EraseOptionBytes();
    Core_begin();
    delay_ms(100);
    F103_GPIO_pinMode_output(LED_PORT, LED_PIN, GPIO_Mode_Out_PP | LED_PIN_SPEED);
    lcd.begin();
    delay_ms(100);
    lcd.clearDisplay();   
    lcd.setRotation(3);



    MODE = 1;
    while (1) {
        if (MODE == 1) {
            menu();
        } else if (MODE == 2) {
            game1();
        } else if (MODE == 3) {
            game2();
        } else if (MODE == 4) {
            game3();
        } else if (MODE == 5) {
            setting();
        }
    }
}
