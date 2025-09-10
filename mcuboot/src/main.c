#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void)
{
    printk("Hello World from Zephyr, booted by MCUboot!\n");
    while (1) {
        k_sleep(K_SECONDS(5));
        printk("Still running...\n");
    }
}