#include <assert.h>
#include <string.h>
#include "uart_hal_stub.h"
#include "uart.h"
#include "kmbox_cmd.h"
#include "actions.h"
#include "icc.h"
struct uart_hal hal;
static uint32_t now;
static unsigned injection_records;
bool icc_send_to_v5f(const icc_record_t *r) {(void)r; injection_records++; return true;}
void USART1_IRQHandler(void);
uint32_t millis(void) {return now;}
void NVIC_EnableIRQ(unsigned irq) {
    (void)irq;hal.irq_enabled = true;
    if (hal.loss_on_enable) {
        hal.loss_on_enable = false;
        hal.input='X'; hal.uart.STATR=USART_FLAG_RXNE | 8;
        USART1_IRQHandler();
    }
}
void NVIC_DisableIRQ(unsigned irq) {
    (void)irq;
    if (hal.loss_on_disable) {
        hal.loss_on_disable = false;
        hal.input = 'X'; hal.uart.STATR = USART_FLAG_RXNE | 2;
        USART1_IRQHandler(); // fault arriving just before the critical section
    }
    hal.irq_enabled = false;
}
static void receive(uint8_t b, uint16_t errors) {
    assert(hal.irq_enabled);
    hal.input=b; hal.uart.STATR=USART_FLAG_RXNE | errors; USART1_IRQHandler();
}
static void drain(unsigned n) {while(n--) {hal.tx_budget=1; USART1_IRQHandler();}}
int main(void) {
    uart_init(921600);
    uint8_t data[4090]; memset(data, 0xA5, sizeof(data));
    assert(uart_tx_write(data, sizeof(data)) == sizeof(data));
    assert(uart_tx_write((const uint8_t *)"TRUNCATE",8) == 0);
    drain(sizeof(data));
    assert(hal.output_len == sizeof(data));
    for (unsigned i=0;i<sizeof(data);i++) assert(hal.output[i]==0xA5);
    assert(uart_tx_write((const uint8_t *)"WRAP",4)==4); drain(4);
    assert(memcmp(hal.output+sizeof(data),"WRAP",4)==0);
    assert(uart_tx_byte_count()==4094);
    uart_set_baud(115200); assert(uart_current_baud()==921600);
    uart_tx_flush(); assert(uart_current_baud()==921600); // TC not TXE controls the change
    hal.tx_complete=true; uart_tx_flush(); assert(uart_current_baud()==115200);
    uart_set_baud(0); assert(uart_current_baud()==115200);

    uint8_t out[256];
    receive('A',0); receive('B',0);
    assert(uart_rx_read(out,sizeof(out))==2 && out[0]=='A' && out[1]=='B');
    for(unsigned i=0;i<1024;i++) receive((uint8_t)i,0);
    assert(uart_rx_drop()==1 && uart_rx_take_loss() && !uart_rx_take_loss());
    assert(uart_rx_read(out,sizeof(out))==0 && uart_rx_available()==0);
    now=24; receive('Z',0); now=48;
    assert(uart_rx_read(out,sizeof(out))==0); // quiet interval restarts on every byte
    now=49; assert(uart_rx_read(out,sizeof(out))==0);
    receive('K',0); assert(uart_rx_read(out,sizeof(out))==1 && out[0]=='K');
    receive('A',0); receive('B',8);
    assert(uart_overrun()==1 && uart_rx_take_loss());
    assert(uart_rx_read(out,sizeof(out))==0);
    now+=25; hal.uart.STATR=USART_FLAG_RXNE;
    assert(uart_rx_read(out,sizeof(out))==0); // a pending hardware byte is not idle
    USART1_IRQHandler(); now+=25; uart_rx_read(out,sizeof(out));
    receive('C',0); hal.loss_on_disable=true;
    assert(uart_rx_read(out,sizeof(out))==0 && uart_rx_take_loss());
    assert(uart_framing()==1 && hal.irq_enabled);

    uart_init(115200); kmbox_cmd_init();
    const char *partial = "km.left(1";
    while (*partial) receive((uint8_t)*partial++,0);
    kmbox_cmd_poll(); assert(g_buttons==0);
    receive('X',8); now+=25; kmbox_cmd_poll();
    receive(')',0); receive('\n',0); kmbox_cmd_poll();
    assert(g_buttons==0 && injection_records==0); // damaged prefix cannot finish a command
    // Loss can arrive after uart_rx_read's copy guard but before parser feed.
    const char *bad = "bad"; while(*bad) receive((uint8_t)*bad++,0);
    kmbox_cmd_poll();
    const char *tail = "km.left(1)\n";
    while(*tail) receive((uint8_t)*tail++,0);
    hal.loss_on_enable=true; kmbox_cmd_poll();
    assert(g_buttons==0 && injection_records==0);
    now+=25; kmbox_cmd_poll();
    const char *valid = "km.left(1)\n";
    while (*valid) receive((uint8_t)*valid++,0);
    kmbox_cmd_poll(); assert(g_buttons==1 && injection_records==1);
    return 0;
}
