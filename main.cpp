#include "mbed.h"
 
Serial pc(USBTX, USBRX); // tx, rx
 
int main() {
    pc.printf("Hello World!\n");
    while(1) {
        pc.putc(pc.getc() + 1);
    }
}
