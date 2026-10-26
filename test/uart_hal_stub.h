#pragma once
#include <stdint.h>
#include <stdbool.h>
#define WCH_IRQ
#define RESET 0
#define ENABLE 1
#define DISABLE 0
#define CMD_USART_IRQn 1
#define CMD_USART_IRQHandler USART1_IRQHandler
#define CMD_USART_GPIO_PORT 0
#define CMD_USART_TX_PIN 1
#define CMD_USART_RX_PIN 2
#define CMD_USART_TX_PINSRC 1
#define CMD_USART_RX_PINSRC 2
#define CMD_USART_GPIO_AF 7
#define CMD_USART_RCC_HB2_UART 1
#define CMD_USART_GPIO_RCC_HB2 2
#define RCC_HB2Periph_AFIO 4
#define GPIO_Speed_Very_High 1
#define GPIO_Mode_AF_PP 1
#define GPIO_Mode_IN_FLOATING 2
#define USART_WordLength_8b 8
#define USART_StopBits_1 1
#define USART_Parity_No 0
#define USART_HardwareFlowControl_None 0
#define USART_Mode_Tx 1
#define USART_Mode_Rx 2
#define USART_IT_RXNE 1
#define USART_IT_TXE 2
#define USART_FLAG_RXNE 0x20
#define USART_FLAG_TC 0x40
struct uart_regs {uint16_t STATR;};
extern struct uart_hal {
    struct uart_regs uart;
    uint16_t interrupts;
    uint8_t input, output[8192];
    unsigned output_len, tx_budget;
    bool irq_enabled, loss_on_disable, loss_on_enable, tx_complete;
} hal;
#define CMD_USART (&hal.uart)
typedef struct {uint32_t USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity,
    USART_HardwareFlowControl, USART_Mode;} USART_InitTypeDef;
typedef struct {uint32_t GPIO_Pin, GPIO_Speed, GPIO_Mode;} GPIO_InitTypeDef;
static inline void RCC_HB2PeriphClockCmd(unsigned a, int b) {(void)a;(void)b;}
static inline void GPIO_PinAFConfig(unsigned a, unsigned b, unsigned c) {(void)a;(void)b;(void)c;}
static inline void GPIO_Init(unsigned a, GPIO_InitTypeDef *g) {(void)a;(void)g;}
static inline void USART_Init(struct uart_regs *u, USART_InitTypeDef *v) {(void)u;(void)v;hal.interrupts=0;}
static inline void USART_Cmd(struct uart_regs *u, int enable) {(void)u;(void)enable;}
static inline void USART_ITConfig(struct uart_regs *u, unsigned it, int enable) {
    (void)u;
    if(enable) hal.interrupts |= it; else hal.interrupts &= ~it;
}
static inline int USART_GetITStatus(struct uart_regs *u, unsigned it) {
    if (!(hal.interrupts & it)) return 0;
    return it == USART_IT_RXNE ? (u->STATR & USART_FLAG_RXNE) != 0 : hal.tx_budget != 0;
}
static inline int USART_GetFlagStatus(struct uart_regs *u, unsigned flag) {return flag == USART_FLAG_TC ? hal.tx_complete : (u->STATR & flag) != 0;}
static inline uint16_t USART_ReceiveData(struct uart_regs *u) {u->STATR = 0; return hal.input;}
static inline void USART_SendData(struct uart_regs *u, uint8_t b) {(void)u;hal.tx_complete=false;hal.output[hal.output_len++]=b;hal.tx_budget--;}
static inline void NVIC_SetPriority(unsigned irq, unsigned p) {(void)irq;(void)p;}
void NVIC_EnableIRQ(unsigned irq);
void NVIC_DisableIRQ(unsigned irq);
uint32_t millis(void);
