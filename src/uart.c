/* uart.c — USART2 command-link transport for the CH32H417 V3F core.
 *
 * Self-contained, protocol-agnostic byte transport:
 *   RX: DMA1 Channel 6 in circular mode into a power-of-two ring; the software
 *       read cursor (rx_read_pos) chases the DMA write position derived from
 *       DMA_GetCurrDataCounter().
 *   TX: a software ring (tx_ring) drained into DMA1 Channel 7 in normal mode,
 *       re-armed on each transfer-complete (TC) interrupt.
 *
 * StdPeriph symbols verified against vendor/wch/Peripheral/inc/{ch32h417.h,
 * ch32h417_dma.h, ch32h417_usart.h, ch32h417_gpio.h, ch32h417_rcc.h}.
 *
 * Channel mapping (EVT ref §5): USART2_TX = DMA1_Ch7, USART2_RX = DMA1_Ch6.
 */
#include "ch32h417_port.h"
#include "board.h"
#include "uart.h"

#define RX_RING_SZ  1024            /* power of two -> mask 1023 */
#define TX_RING_SZ  4096            /* power of two -> mask 4095 */

static uint8_t  rx_dma[RX_RING_SZ];
static uint16_t rx_read_pos;
static uint32_t rx_total;

static uint8_t  tx_ring[TX_RING_SZ];
static volatile uint16_t tx_head, tx_tail;
static volatile bool     tx_dma_busy;
static volatile uint16_t tx_inflight;   /* bytes programmed into the active TX DMA xfer */
static uint32_t tx_total;
static uint32_t s_baud;
static uint32_t err_or, err_fe, err_ne;

/* DMA channels for the command USART come from board.h (default USART3:
 * TX=DMA1_Ch2, RX=DMA1_Ch3). Keeping the channel/mux/IRQ/ISR all in board.h
 * guarantees the TX ISR name below matches the vector-table entry. */
#define RX_DMA_CH   CMD_RX_DMA_CH
#define TX_DMA_CH   CMD_TX_DMA_CH
#define TX_DMA_IRQn CMD_TX_DMA_IRQn

static void rx_dma_setup(void)
{
    DMA_InitTypeDef d = {0};
    DMA_DeInit(RX_DMA_CH);
    d.DMA_PeripheralBaseAddr = (uint32_t)CMD_USART_DATAR;
    d.DMA_Memory0BaseAddr    = (uint32_t)rx_dma;   /* field is Memory0BaseAddr (see ch32h417_dma.h) */
    d.DMA_DIR                = DMA_DIR_PeripheralSRC;
    d.DMA_BufferSize         = RX_RING_SZ;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Circular;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(RX_DMA_CH, &d);
    DMA_Cmd(RX_DMA_CH, ENABLE);
    rx_read_pos = 0;
}

/* Arm a single contiguous TX DMA burst from tx_tail. We only ever program the
 * span from tx_tail up to either tx_head or the ring end (whichever comes
 * first), so a ring wrap is handled as two back-to-back DMA transfers (the IRQ
 * handler re-kicks for the remainder). The programmed length is recorded in
 * tx_inflight so the IRQ can advance tx_tail by exactly the bytes that drained
 * (DMA_GetCurrDataCounter reads 0 at TC and must NOT be used for this). */
static void tx_dma_kick(void)
{
    if (tx_dma_busy) return;
    uint16_t head = tx_head, tail = tx_tail;
    if (head == tail) return;                 /* nothing to send */
    uint16_t chunk = (head > tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(TX_RING_SZ - tail); /* up to ring end */
    DMA_InitTypeDef d = {0};
    DMA_DeInit(TX_DMA_CH);
    d.DMA_PeripheralBaseAddr = (uint32_t)CMD_USART_DATAR;
    d.DMA_Memory0BaseAddr    = (uint32_t)&tx_ring[tail];
    d.DMA_DIR                = DMA_DIR_PeripheralDST;
    d.DMA_BufferSize         = chunk;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_High;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(TX_DMA_CH, &d);
    tx_inflight = chunk;
    tx_dma_busy = true;
    DMA_ITConfig(TX_DMA_CH, DMA_IT_TC, ENABLE);
    DMA_Cmd(TX_DMA_CH, ENABLE);
}

static void usart_apply(uint32_t baud)
{
    USART_InitTypeDef u = {0};
    u.USART_BaudRate            = baud;
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(CMD_USART, &u);
}

void uart_init(uint32_t baud)
{
    s_baud = baud;
    RCC_HB1PeriphClockCmd(CMD_USART_RCC_HB1, ENABLE);          /* USART2 on HB1 bus */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);           /* DMA1 on HB bus */
    RCC_HB2PeriphClockCmd(CMD_USART_GPIO_RCC_HB2, ENABLE);     /* USART2 GPIO port on HB2 */

    /* GPIO alternate-function for USART2 pins. On this part the pin is routed
     * to the peripheral by the per-pin AFR mux (STM32-style) — GPIO_Mode_AF_PP
     * alone is NOT enough; GPIO_PinAFConfig(..., AF7) MUST select the USART2
     * function or the pad stays on AF0 and no signal reaches the peripheral.
     * Pins/port/AF come from board.h (default USART2 = PD5 TX / PD6 RX, AF7,
     * matching every WCH EVT example). */
    GPIO_PinAFConfig(CMD_USART_GPIO_PORT, CMD_USART_TX_PINSRC, CMD_USART_GPIO_AF);
    GPIO_PinAFConfig(CMD_USART_GPIO_PORT, CMD_USART_RX_PINSRC, CMD_USART_GPIO_AF);

    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = CMD_USART_TX_PIN;           /* USART2 TX */
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;            /* alt-function push-pull */
    GPIO_Init(CMD_USART_GPIO_PORT, &g);

    g.GPIO_Pin   = CMD_USART_RX_PIN;           /* USART2 RX */
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;      /* input floating */
    GPIO_Init(CMD_USART_GPIO_PORT, &g);

    usart_apply(baud);

    /* Route the DMA request lines to our channels via the DMAMUX. Without this
     * the channels stay muxed to request source 0 (reset default), so USART2
     * RX/TX DMA requests never reach DMA1 Ch6/Ch7 and the link is dead in both
     * directions. (EVT USART_DMA Common/hardware.c:188-189.) Mux channel index
     * == DMA channel number: Ch7=TX(req 87), Ch6=RX(req 88). */
    DMA_MuxChannelConfig(CMD_TX_DMA_MUX, CMD_USART_DMA_REQ_TX);   /* cmd USART TX */
    DMA_MuxChannelConfig(CMD_RX_DMA_MUX, CMD_USART_DMA_REQ_RX);   /* cmd USART RX */

    USART_DMACmd(CMD_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
    USART_Cmd(CMD_USART, ENABLE);

    rx_dma_setup();
    tx_head = tx_tail = 0;
    tx_dma_busy = false;
    tx_inflight = 0;
    NVIC_SetPriority(TX_DMA_IRQn, 32);
    NVIC_EnableIRQ(TX_DMA_IRQn);
}

void uart_set_baud(uint32_t baud)
{
    s_baud = baud;
    USART_Cmd(CMD_USART, DISABLE);
    usart_apply(baud);
    USART_Cmd(CMD_USART, ENABLE);
}

uint32_t uart_current_baud(void) { return s_baud; }

uint16_t uart_rx_available(void)
{
    /* DMA writes forward; CurrDataCounter counts DOWN from RX_RING_SZ. */
    uint16_t dma_pos = (uint16_t)(RX_RING_SZ - DMA_GetCurrDataCounter(RX_DMA_CH));
    return (uint16_t)((dma_pos - rx_read_pos) & (RX_RING_SZ - 1));
}

uint16_t uart_rx_read(uint8_t *dst, uint16_t max)
{
    uint16_t n = 0;
    uint16_t dma_pos = (uint16_t)(RX_RING_SZ - DMA_GetCurrDataCounter(RX_DMA_CH));
    while (rx_read_pos != dma_pos && n < max) {
        dst[n++] = rx_dma[rx_read_pos];
        rx_read_pos = (uint16_t)((rx_read_pos + 1) & (RX_RING_SZ - 1));
    }
    rx_total += n;
    return n;
}

uint16_t uart_tx_room(void)
{
    uint16_t used = (uint16_t)((tx_head - tx_tail) & (TX_RING_SZ - 1));
    return (uint16_t)(TX_RING_SZ - 1 - used);
}

uint16_t uart_tx_write(const uint8_t *src, uint16_t len)
{
    uint16_t n = 0;
    while (n < len && uart_tx_room() > 0) {
        tx_ring[tx_head] = src[n++];
        tx_head = (uint16_t)((tx_head + 1) & (TX_RING_SZ - 1));
    }
    tx_total += n;
    tx_dma_kick();
    return n;
}

void uart_tx_flush(void) { tx_dma_kick(); }

uint32_t uart_overrun(void)       { return err_or; }
uint32_t uart_framing(void)       { return err_fe; }
uint32_t uart_noise(void)         { return err_ne; }
uint32_t uart_rx_byte_count(void) { return rx_total; }
uint32_t uart_tx_byte_count(void) { return tx_total; }

/* TX DMA transfer-complete handler. The ISR name + TC flag come from board.h
 * (CMD_TX_DMA_IRQHandler / CMD_TX_DMA_IT_TC) so they always match the TX DMA
 * channel and the vector-table entry in core/startup_v3f.S — change the channel
 * in board.h and this handler retargets with it (no silent ISR/vector drift).
 * Default USART3 -> DMA1_Channel2_IRQHandler / DMA1_IT_TC2.
 * DMA_GetITStatus/ClearITPendingBit take a DMA_TypeDef* (DMA1) per ch32h417_dma.h. */
void CMD_TX_DMA_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void CMD_TX_DMA_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, CMD_TX_DMA_IT_TC)) {
        DMA_ClearITPendingBit(DMA1, CMD_TX_DMA_IT_TC);
        DMA_Cmd(TX_DMA_CH, DISABLE);
        /* Free exactly the bytes we programmed for this burst. The DMA current
         * data counter reads 0 at TC and MUST NOT be used here. */
        tx_tail = (uint16_t)((tx_tail + tx_inflight) & (TX_RING_SZ - 1));
        tx_inflight = 0;
        tx_dma_busy = false;
        tx_dma_kick();   /* send next chunk (e.g. ring-wrap remainder or newly queued bytes) */
    }
}
