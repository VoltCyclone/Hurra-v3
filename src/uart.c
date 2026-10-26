/* uart.c — command-link USART transport for the CH32H417 V3F core.
 *
 * Protocol-agnostic byte transport, interrupt-driven (no DMA):
 *   RX: RXNE interrupt pushes each byte into a power-of-two ring drained by
 *       uart_rx_read().
 *   TX: uart_tx_write() fills a ring and enables the TXE interrupt, which sends
 *       one byte per TXE until empty, then masks itself.
 *
 * Port/pins/baud come from board.h. Default: WCH-LinkE virtual COM on USART1
 * PA9(TX)/PA10(RX) AF7. The IRQ handler name (CMD_USART_IRQHandler) comes from
 * board.h to match the vector-table entry in core/startup_v3f.S.
 */
#ifdef UART_HOSTTEST
#include "uart_hal_stub.h"
#else
#include "ch32h417_port.h"
#include "board.h"
#include "timebase.h"
#endif
#include "uart.h"
#include "uart_rx_class.h"

#define RX_RING_SZ  1024            /* power of two -> mask 1023 */
#define TX_RING_SZ  4096            /* power of two -> mask 4095 */

static volatile uint8_t  rx_ring[RX_RING_SZ];
static volatile uint16_t rx_head;   /* ISR writes */
static volatile uint16_t rx_tail;   /* uart_rx_read() reads */
static uint32_t rx_total;

static volatile uint8_t  tx_ring[TX_RING_SZ];
static volatile uint16_t tx_head;   /* uart_tx_write() writes */
static volatile uint16_t tx_tail;   /* ISR reads */
static uint32_t tx_total;

static uint32_t s_baud;
static uint32_t s_pending_baud;
/* err_or = hardware overrun (STATR.ORE); err_fe/err_ne = framing/noise.
 * rx_drop counts bytes lost to a full RX ring — kept distinct so a software
 * back-pressure drop is never miscounted as a hardware overrun. */
static volatile uint32_t err_or, err_fe, err_ne, rx_drop;
static volatile uint32_t rx_loss_epoch, rx_last_byte_ms;
static volatile bool rx_recovering;
static uint32_t rx_reported_epoch;
#define RX_RECOVERY_IDLE_MS 25u

static void rx_lost(void)
{
    if (!rx_recovering) rx_loss_epoch++;
    rx_recovering = true;
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
    s_pending_baud = 0;
    /* USART1 clock + its GPIO port (PA9/PA10) + AFIO are all on the HB2 bus. */
    RCC_HB2PeriphClockCmd(CMD_USART_RCC_HB2_UART | CMD_USART_GPIO_RCC_HB2 |
                          RCC_HB2Periph_AFIO, ENABLE);

    /* Per-pin AFR mux: GPIO_Mode_AF_PP alone isn't enough — select AF7 or the
     * pad stays on AF0 and no signal reaches USART1. (EVT debug.c PA9 AF7.) */
    GPIO_PinAFConfig(CMD_USART_GPIO_PORT, CMD_USART_TX_PINSRC, CMD_USART_GPIO_AF);
    GPIO_PinAFConfig(CMD_USART_GPIO_PORT, CMD_USART_RX_PINSRC, CMD_USART_GPIO_AF);

    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = CMD_USART_TX_PIN;           /* TX = PA9 */
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;            /* alt-function push-pull */
    GPIO_Init(CMD_USART_GPIO_PORT, &g);

    g.GPIO_Pin   = CMD_USART_RX_PIN;           /* RX = PA10 */
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;      /* input floating */
    GPIO_Init(CMD_USART_GPIO_PORT, &g);

    usart_apply(baud);

    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    rx_total = tx_total = 0;
    err_or = err_fe = err_ne = rx_drop = 0;
    rx_loss_epoch = rx_reported_epoch = 0;
    rx_recovering = false;
    rx_last_byte_ms = millis();

    /* RX interrupt on every received byte; TX (TXE) interrupt enabled on demand
     * by uart_tx_write() so an empty TX ring doesn't spin the ISR. */
    USART_ITConfig(CMD_USART, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(CMD_USART_IRQn, 16);
    NVIC_EnableIRQ(CMD_USART_IRQn);

    USART_Cmd(CMD_USART, ENABLE);
}

void uart_set_baud(uint32_t baud)
{
    if (!baud) return;
    s_pending_baud = baud;
    uart_tx_flush(); // apply only once queued bytes AND the wire have drained
}

uint32_t uart_current_baud(void) { return s_baud; }

uint16_t uart_rx_available(void)
{
    if (rx_recovering) return 0;
    return (uint16_t)((rx_head - rx_tail) & (RX_RING_SZ - 1));
}

uint16_t uart_rx_read(uint8_t *dst, uint16_t max)
{
    if (!dst || !max) return 0;
    if (rx_recovering) {
        // Foreground-only API: briefly exclude this USART IRQ while discarding
        // the damaged prefix. Do not mask timer/USB interrupts or copy here.
        NVIC_DisableIRQ(CMD_USART_IRQn);
        rx_tail = rx_head;
        if (millis() - rx_last_byte_ms >= RX_RECOVERY_IDLE_MS &&
            USART_GetFlagStatus(CMD_USART, USART_FLAG_RXNE) == RESET)
            rx_recovering = false;
        NVIC_EnableIRQ(CMD_USART_IRQn);
        return 0;
    }
    uint16_t n = 0;
    while (n < max && !rx_recovering) {
        uint16_t head = rx_head;                 /* snapshot ISR-updated head */
        if (rx_tail == head) break;              /* empty */
        dst[n++] = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) & (RX_RING_SZ - 1));
    }
    // A loss interrupt during copying invalidates this batch as a whole.
    NVIC_DisableIRQ(CMD_USART_IRQn);
    if (rx_recovering) n = 0;
    rx_total += n;
    NVIC_EnableIRQ(CMD_USART_IRQn);
    return n;
}

bool uart_rx_take_loss(void)
{
    uint32_t epoch = rx_loss_epoch;
    bool changed = epoch != rx_reported_epoch;
    rx_reported_epoch = epoch;
    return changed;
}

uint16_t uart_tx_room(void)
{
    uint16_t used = (uint16_t)((tx_head - tx_tail) & (TX_RING_SZ - 1));
    return (uint16_t)(TX_RING_SZ - 1 - used);
}

uint16_t uart_tx_write(const uint8_t *src, uint16_t len)
{
    // One foreground producer; the ISR can only free more room. Publish the
    // head once, after the complete write, so a full ring never emits a prefix.
    if (!src || len > uart_tx_room()) return 0;
    uint16_t head = tx_head;
    for (uint16_t i = 0; i < len; i++) {
        tx_ring[head] = src[i];
        head = (uint16_t)((head + 1) & (TX_RING_SZ - 1));
    }
#ifdef __riscv
    __asm volatile("fence rw,rw" ::: "memory");
#else
    __asm volatile("" ::: "memory");
#endif
    tx_head = head;
    tx_total += len;
    if (len) USART_ITConfig(CMD_USART, USART_IT_TXE, ENABLE);
    return len;
}

void uart_tx_flush(void)
{
    if (tx_head != tx_tail) {
        USART_ITConfig(CMD_USART, USART_IT_TXE, ENABLE);
    } else if (s_pending_baud && USART_GetFlagStatus(CMD_USART, USART_FLAG_TC) != RESET) {
        // TXE/empty software ring is not enough: the last stop bit must leave
        // the shifter before changing the divisor. No waiting with IRQs masked.
        uint32_t baud = s_pending_baud;
        s_pending_baud = 0;
        USART_Cmd(CMD_USART, DISABLE);
        usart_apply(baud);
        USART_ITConfig(CMD_USART, USART_IT_RXNE, ENABLE);
        USART_Cmd(CMD_USART, ENABLE);
        s_baud = baud;
    }
}

uint32_t uart_overrun(void)       { return err_or; }
uint32_t uart_framing(void)       { return err_fe; }
uint32_t uart_noise(void)         { return err_ne; }
uint32_t uart_rx_drop(void)       { return rx_drop; }
uint32_t uart_rx_byte_count(void) { return rx_total; }
uint32_t uart_tx_byte_count(void) { return tx_total; }

/* Single USART IRQ: RXNE (byte received) + TXE (ready for next TX byte). */
void CMD_USART_IRQHandler(void) WCH_IRQ;
void CMD_USART_IRQHandler(void)
{
    /* RX: drain RXNE into the ring. Snapshot STATR BEFORE reading DATAR — on
     * this part ORE/FE/NE are cleared by the "read STATR then read DATAR"
     * sequence, so reading DATAR first would lose the error indication. Reading
     * DATAR always (even on error) clears RXNE/ORE so the link can't wedge. */
    if (USART_GetITStatus(CMD_USART, USART_IT_RXNE) != RESET) {
        uint16_t sr = (uint16_t)CMD_USART->STATR;          /* snapshot first */
        uint8_t  b  = (uint8_t)USART_ReceiveData(CMD_USART); /* clears RXNE/ORE */
        rx_last_byte_ms = millis();
        uart_rx_class_t c = uart_rx_classify(sr);
        if (c.count_or) err_or++;
        if (c.count_fe) err_fe++;
        if (c.count_ne) err_ne++;
        if (c.count_or || c.drop) rx_lost();
        if (!rx_recovering) {                           /* FE/NE byte is corrupt -> skip */
            uint16_t next = (uint16_t)((rx_head + 1) & (RX_RING_SZ - 1));
            if (next != rx_tail) {               /* drop on full (don't clobber) */
                rx_ring[rx_head] = b;
                rx_head = next;
            } else {
                rx_drop++;                       /* ring full -> software drop */
                rx_lost();
            }
        }
    }

    /* TX: feed one byte per TXE; mask the interrupt when the ring is empty so an
     * idle link doesn't re-enter the ISR forever. */
    if (USART_GetITStatus(CMD_USART, USART_IT_TXE) != RESET) {
        if (tx_tail != tx_head) {
            USART_SendData(CMD_USART, tx_ring[tx_tail]);   /* writing DATAR clears TXE */
            tx_tail = (uint16_t)((tx_tail + 1) & (TX_RING_SZ - 1));
        } else {
            USART_ITConfig(CMD_USART, USART_IT_TXE, DISABLE);
        }
    }
}
