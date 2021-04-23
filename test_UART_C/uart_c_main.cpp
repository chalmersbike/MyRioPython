#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "bbb_uart.h"

using namespace std;

int main(){
    UART uart(1,460800,BOTH,false);

//    char write_str[] = "Hello world !";
    char write_str[] = {'H','e','!'};
    cout << "write_str : ";
    for (int i=0;i<sizeof(write_str);i++){
        cout << write_str[i];
    }
    cout << endl;
    cout << "sizeof(write_str) : " << sizeof(write_str) << endl;

    int* uart_read_buf = (int*) malloc(sizeof(write_str));
    int* uart_read_buf2 = (int*) malloc(sizeof(write_str));
    int number_bytes_read = 0;
    int number_bytes_read2 = 0;

    uart.init();

//    uart.uart_write((void*) write_str, sizeof(write_str));
    uart.uart_write(write_str, sizeof(write_str));
    uart.uart_write(&write_str, sizeof(write_str));

//    number_bytes_read = uart.uart_read((void*) uart_read_buf, sizeof(write_str));
//    number_bytes_read2 = uart.uart_read(uart_read_buf2, sizeof(write_str));
    number_bytes_read2 = uart.uart_read(&uart_read_buf2, sizeof(write_str));
    cout << "number_bytes_read : " << number_bytes_read << endl;
    cout << "number_bytes_read2 : " << number_bytes_read2 << endl;
    cout << "uart_read_buf : ";
    for (int i=0;i<sizeof(write_str);i++){
        cout << uart_read_buf[i];
    }
    cout << endl;
    cout << "uart_read_buf2 : ";
    for (int i=0;i<sizeof(write_str);i++){
        cout << uart_read_buf2[i];
    }
    cout << endl;

    free(uart_read_buf);
    free(uart_read_buf2);
}