/* Calls the team's UI_print.c TCB API, flushes the queue like a scheduler would, captures UART bytes to frames.bin. */
#include <stdio.h>
#include "stm32f4xx.h"
#include "UI_print.h"
UART_HandleTypeDef huart6;
extern UI_TCB *UI_SendTCBSequence[30];
extern int UI_TCBNum;
static FILE *out;
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) { fwrite(d, 1, n, out); return 0; }

static void flush_all(void) {
    for (int i = 0; i < UI_TCBNum; i++) UI_Draw(UI_SendTCBSequence[i]->senddata, (uint16_t)UI_SendTCBSequence[i]->size);
}

int main(void) {
    printf("sizeof graphic=%zu int=%zu char=%zu head=%zu\n", sizeof(graphic_data_struct_t),
           sizeof(int_data_struct_t), sizeof(ext_client_custom_character_t), sizeof(Head));
    out = fopen("frames.bin", "wb");
    uiDrawGraphicsSeq(LINE, "c01", 1, 0, 1, 2, 0, 0, 940, 540, 980, 540, 0, 10);
    uiDrawGraphicsSeq(LINE, "c02", 1, 0, 1, 2, 0, 0, 960, 520, 960, 560, 0, 10);
    uiDrawGraphicsSeq(CIRCLE, "c03", 1, 0, 2, 2, 0, 0, 960, 540, 0, 0, 30, 10);
    uiDrawGraphicsSeq(RECTANGLE, "r01", 1, 1, 6, 2, 0, 0, 1500, 200, 1800, 260, 0, 10);
    uiDrawGraphicsSeq(ARC, "a01", 1, 1, 3, 6, 30, 150, 960, 540, 200, 200, 0, 10);
    uiDrawGraphicsSeq(ELLIPSE, "e01", 1, 1, 4, 2, 0, 0, 300, 300, 80, 40, 0, 10);
    UI_TCB *num = uiDrawIntDataSeq("i01", 1, 2, 8, 20, 2, 1520, 245, 1234, 10);
    uiDrawFloatDataSeq("f01", 1, 2, 2, 20, 2, 2, 1520, 300, 12345, 10);
    UI_TCB *str = uiDrawStringDataSeq("s01", 1, 3, 8, 20, 2, 100, 800, (uint8_t *)"HNU NHS UI TEST", 10);
    flush_all();
    /* second pass: what the refresh helpers produce for a live value + string */
    num->FOA = 2; str->FOA = 2;
    uiRefreshIntDataSeq(num, num, 2, 8, 20, 2, 1520, 245, 999, 10);
    uiRefreshStringDataSeq(str, str, 3, 8, 20, 2, 100, 800, (uint8_t *)"REFRESHED STR", 10);
    UI_Draw(num->senddata, (uint16_t)num->size);
    UI_Draw(str->senddata, (uint16_t)str->size);
    fclose(out);
    printf("ok tcb=%d\n", UI_TCBNum);
    return 0;
}
