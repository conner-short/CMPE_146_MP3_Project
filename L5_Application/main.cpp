#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "LPC17xx.h"

#include "ff.h"
#include "pin_t.hpp"
#include "source/cmd_handlers/mp3Handler.hpp"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"

#include "LabUART.hpp"
#include "Scroll_Nav.hpp"
#include "SPIController.hpp"
#include "VS1053.hpp"

static const uint8_t SCREEN_REFRESH = 0x0C;
static const uint8_t SCREEN_INIT[3] = {0x16, SCREEN_REFRESH, 0x11};
static const uint8_t NO_CURSOR[2] = {' ', ' '};
static const uint8_t CURSOR[2]    = {'>', ' '};
static const uint8_t LINE0_START  = 0x80;
static const uint8_t LINE1_START  = 0x94;
static const uint8_t LINE2_START  = 0xA8;
static const uint8_t LINE3_START  = 0xBC;

typedef struct
{
    VS1053* dec;
    Scroll_Nav* nav;
} task_params_t;

FRESULT getMP3PathRec(bool* ret, uint32_t* index, uint8_t* buf, uint32_t len)
{
    FRESULT ff_res;
    DIR dir;
    static FILINFO file_info; /* Static to save on memory */

    *ret = false;

    /* Start scanning for MP3 files */

    ff_res = f_opendir(&dir, (char*)buf);

    if(ff_res == FR_OK)
    {
        while(1)
        {
            ff_res = f_readdir(&dir, &file_info);

            if((ff_res != FR_OK) || (file_info.fname[0] == '\0'))
            {
                *ret = false;
                break;
            }

            /* File is a directory, recurse in to it */
            if(file_info.fattrib & AM_DIR)
            {
                UINT i = strlen((char*)buf);
                snprintf((char*)(&(buf[i])), len - strlen((char*)(&(buf[i]))), "\\%s", file_info.fname);
                ff_res = getMP3PathRec(ret, index, buf, len);

                if(ff_res != FR_OK)
                {
                    *ret = false;
                    break;
                }

                else if(*ret == true)
                {
                    /* Found the file, no need to continue */
                    break;
                }

                else
                {
                    buf[i] = '\0'; /* Remove the previously-appended directory */
                }
            }

            /* Regular file, check if it's an MP3 file */
            else if((strlen(file_info.fname) >= 3) &&
                        (strncmp(&(file_info.fname[strlen(file_info.fname) - 3]), "MP3", 3) == 0))
            {
                if(*index == 0)
                {
                    /* Found the file, append the name to the path */
                    UINT i = strlen((char*)buf);
                    snprintf((char*)(&(buf[i])), len - strlen((char*)(&(buf[i]))), "\\%s", file_info.fname);

                    *ret = true;

                    break;
                }

                else
                {
                    (*index)--;
                }
            }
        }

        f_closedir(&dir);
    }
    else
    {
        *ret = false;
        return ff_res;
    }

    return ff_res;
}

bool getMP3Path(uint32_t index, uint8_t* buf, uint32_t len)
{
    /* Make sure we have enough room to work */
    if(len < 256)
    {
        return false;
    }

    bool ret;

    strncpy((char*)buf, "1:", len);

    getMP3PathRec(&ret, &index, buf, len);

    return ret;
}

void drawMenuLine(uint8_t line_num, uint8_t* line, bool cursor)
{
    LabUART& uart = LabUART::getInstance();

    uint8_t line_copy[19];
    line_copy[18] = '\0';
    strncpy((char*)line_copy, (char*)line, 18);

    uint32_t len = strlen((char*)line_copy);

    /* Per datasheet */
    for(uint32_t i = 0; i < len; i++)
    {
        if(line_copy[i] == '\\')
        {
            line_copy[i] = 0;
        }
        else if(line_copy[i] == '~')
        {
            line_copy[i] = 1;
        }
    }

    switch(line_num)
    {
        case 0: uart.transmit(LabUART::UART3, &LINE0_START, 1); break;
        case 1: uart.transmit(LabUART::UART3, &LINE1_START, 1); break;
        case 2: uart.transmit(LabUART::UART3, &LINE2_START, 1); break;
        case 3: uart.transmit(LabUART::UART3, &LINE3_START, 1); break;
        default: return;
    }

    uart.transmit(LabUART::UART3, cursor ? CURSOR : NO_CURSOR, 2);

    if(len < 18)
    {
        uart.transmit(LabUART::UART3, line_copy, len);
    }
    else
    {
        uart.transmit(LabUART::UART3, line_copy, 18);
    }
}

void drawMenu(uint8_t* line0, uint8_t* line1, uint8_t* line2, uint8_t* line3, int cursor_pos)
{
    LabUART& uart = LabUART::getInstance();

    uart.transmit(LabUART::UART3, &SCREEN_REFRESH, 1);
    vTaskDelay(5);

    drawMenuLine(0, line0, cursor_pos == 0);
    drawMenuLine(1, line1, cursor_pos == 1);
    drawMenuLine(2, line2, cursor_pos == 2);
    drawMenuLine(3, line3, cursor_pos == 3);
}

void appTask(void* p)
{
    VS1053* dec = ((task_params_t*)p)->dec;
    Scroll_Nav* nav = ((task_params_t*)p)->nav;

    uint8_t vol = 0x18;

    enum {MENU, PLAYER} mode = MENU;

    uint8_t menu_file_paths[4][256];
    int menu_lines;

    /* Do initial population of menu */

    for(menu_lines = 0; menu_lines < 4; menu_lines++)
    {
        if(!getMP3Path(menu_lines, menu_file_paths[menu_lines], 256))
        {
            break;
        }
    }

    /* If there aren't enough files to fill the whole list (4 items), nullify
     * the remaining entries */
    for(; menu_lines < 4; menu_lines++)
    {
        menu_file_paths[menu_lines][0] = '\0';
    }

    uint8_t menu_buf_index = 0, cursor_pos = 0;

    /* Initialize the UART for communicating with the screen */
    LabUART& uart = LabUART::getInstance();
    uart.init(LabUART::UART3, 8, LabUART::PARITY_NONE, 1, 19200);

    uart.transmit(LabUART::UART3, SCREEN_INIT, 3);

    vTaskDelay(5); /* Per screen datasheet */

    drawMenu(
        menu_file_paths[(menu_buf_index + 0) % 4],
        menu_file_paths[(menu_buf_index + 1) % 4],
        menu_file_paths[(menu_buf_index + 2) % 4],
        menu_file_paths[(menu_buf_index + 3) % 4],
        cursor_pos
    );

    uint8_t path_buf[256];

    while(1)
    {
        switch(nav->waitForNextEvent(500))
        {
            case Scroll_Nav::NONE:
                /* Waiting timed out */
                break;

            case Scroll_Nav::WHEEL_CW:
                switch(mode)
                {
                    case MENU:
                        if(cursor_pos == 3)
                        {
                            /* Get next line */

                            menu_buf_index++;

                            if(!getMP3Path(menu_buf_index + 3, path_buf, 256))
                            {
                                /* Reached the end, keep everything the same */
                                menu_buf_index--;
                            }
                            else
                            {
                                memcpy(menu_file_paths[(menu_buf_index + 3) % 4], path_buf, 256);
                            }
                        }
                        else if(cursor_pos < (menu_lines - 1))
                        {
                            /* Move the cursor */
                            cursor_pos++;
                        }

                        drawMenu(
                            menu_file_paths[(menu_buf_index + 0) % 4],
                            menu_file_paths[(menu_buf_index + 1) % 4],
                            menu_file_paths[(menu_buf_index + 2) % 4],
                            menu_file_paths[(menu_buf_index + 3) % 4],
                            cursor_pos
                        );
                        break;

                    case PLAYER:
                        /* Increase the volume */
                        if(vol >= 0x06)
                        {
                            vol -= 0x06;
                            dec->setVolume(vol);
                        }
                        break;
                }
                break;

            case Scroll_Nav::WHEEL_CCW:
                switch(mode)
                {
                    case MENU:
                        if(cursor_pos == 0)
                        {
                            /* Get previous line */

                            menu_buf_index -= (menu_buf_index > 0) ? 1 : 0; /* Beware of underflow */

                            if(!getMP3Path(menu_buf_index, path_buf, 256))
                            {
                                /* Reached the end, keep everything the same */
                                menu_buf_index--;
                            }

                            else
                            {
                                memcpy(menu_file_paths[(menu_buf_index + 0) % 4], path_buf, 256);
                            }
                        }
                        else
                        {
                            /* Move the cursor */
                            cursor_pos--;
                        }

                        drawMenu(
                            menu_file_paths[(menu_buf_index + 0) % 4],
                            menu_file_paths[(menu_buf_index + 1) % 4],
                            menu_file_paths[(menu_buf_index + 2) % 4],
                            menu_file_paths[(menu_buf_index + 3) % 4],
                            cursor_pos
                        );
                        break;

                    case PLAYER:
                        /* Decrease the volume */
                        if(vol < 0xFE)
                        {
                            vol += 0x06;
                            dec->setVolume(vol);
                        }
                        break;
                }
                break;

            case Scroll_Nav::S1_DOWN:
                break;

            case Scroll_Nav::S1_UP:
                break;

            case Scroll_Nav::S2_DOWN:
                break;

            case Scroll_Nav::S2_UP:
                break;

            case Scroll_Nav::S3_DOWN:
                break;

            case Scroll_Nav::S3_UP:
                break;

            case Scroll_Nav::S4_DOWN:
                break;

            case Scroll_Nav::S4_UP:
                break;

            case Scroll_Nav::S5_DOWN:
                break;

            case Scroll_Nav::S5_UP:
                break;

            default: break;
        }
    }
}

int main(void) {
    static VS1053 mp3Decoder;
    pin_t reset = {2, 4};
    pin_t dreq = {2, 5};
    pin_t control_cs = {2, 6};
    pin_t data_cs = {2, 7};

    /* Configure SPI pin functions */
    LPC_PINCON->PINSEL0 &= ~(3 << 30);
    LPC_PINCON->PINSEL0 |=  (2 << 30);
    LPC_PINCON->PINSEL1 &= ~((3 << 4) | (3 << 2));
    LPC_PINCON->PINSEL1 |=  ((2 << 4) | (2 << 2));

    if(!mp3Decoder.init(SPIController::SSP0, reset, data_cs, control_cs, dreq))
    {
        uart0_puts("Failed to initialize decoder");

        while(1)
        {
            ;
        }
    }

    static Scroll_Nav nav;

    pin_t s2 = {2, 1};
    pin_t s3 = {2, 2};
    pin_t s4 = {2, 3};
    pin_t s5 = {0, 0};
    pin_t wheel_a = {0, 29};
    pin_t wheel_b = {0, 30};

    if(!nav.init(NULL, &s2, &s3, &s4, &s5, &wheel_a, &wheel_b))
    {
        uart0_puts("Failed to initialize navigation buttons");

        while(1)
        {
            ;
        }
    }

    mp3CmdDec = &mp3Decoder;

    static task_params_t params;
    params.dec = &mp3Decoder;
    params.nav = &nav;

    scheduler_add_task(new terminalTask(1));

    xTaskCreate(appTask, "UI/App", 1024, &params, 1, NULL);

    scheduler_start();

    return 0;
}
