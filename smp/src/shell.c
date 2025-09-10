#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/version.h>
#include "common.h"


static int cmd_sensor(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    shell_print(shell, "Starting sensor thread on UART...");
    start_smp_sensor(shell);
    return 0;
}

static int cmd_sensor_once(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    char buf[64];
    int rc = sensor_read_once(buf, sizeof(buf));
    if (rc == 0) {
        shell_print(shell, "%s", buf);
    } else {
        // still print what we have (e.g., STATUS or error text)
        shell_print(shell, "%s (rc=%d)", buf, rc);
    }
    return 0;
}

static int cmd_exit_sensor(const struct shell *shell, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    shell_print(shell, "Stopping sensor thread (exit)...");
    stop_smp_sensor();
    return 0;
}

static int cmd_hello(const struct shell *shell, size_t argc, char **argv) {
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    shell_print(shell, "Hello, Zephyr Shell!");
    return 0;
}
static int cmd_listall(const struct shell *shell, size_t argc, char **argv) {
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    shell_print(shell, "\n=== Device Information ===");
    // System Information
    shell_print(shell, "\n--- System Info ---");
    shell_print(shell, "Board: %s", CONFIG_BOARD);
    shell_print(shell, "SoC: %s", CONFIG_SOC);
    shell_print(shell, "Zephyr Version: %s", KERNEL_VERSION_STRING);
    shell_print(shell, "Build Time: %s %s", __DATE__, __TIME__);
    // Runtime Information
    shell_print(shell, "\n--- Runtime Info ---");
    shell_print(shell, "Uptime: %llu ms", (unsigned long long)k_uptime_get());
    shell_print(shell, "CPU Frequency: %d Hz", sys_clock_hw_cycles_per_sec());

#ifdef CONFIG_NETWORKING
    shell_print(shell, "\n--- Network Info ---");
    shell_print(shell, "Networking: Enabled");

#ifdef CONFIG_MCUMGR_TRANSPORT_UDP
    shell_print(shell, "UDP Transport: Enabled (Port 1337)");
#endif
#else
    shell_print(shell, "\n--- Network Info ---");
    shell_print(shell, "Networking: Disabled");
#endif

    // MCUmgr Information
    shell_print(shell, "\n--- MCUmgr Info ---");
#ifdef CONFIG_MCUMGR_TRANSPORT_UART
    shell_print(shell, "UART Transport: Enabled");
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
    shell_print(shell, "Bluetooth Transport: Enabled");
#endif

    // Hardware Information
    shell_print(shell, "--- Hardware Info ---");
    shell_print(shell, "Board: %s", CONFIG_BOARD);
    shell_print(shell, "=== End Device Info ===");

    return 0;
}


// Register shell commands
SHELL_CMD_REGISTER(hello, NULL, "Say hello", cmd_hello);
SHELL_CMD_REGISTER(listall, NULL, "List all device information", cmd_listall);
SHELL_CMD_REGISTER(sensor, NULL, "Start sensor task", cmd_sensor);
SHELL_CMD_REGISTER(sensoronce, NULL, "Read sensor once and print immediately", cmd_sensor_once);
SHELL_CMD_REGISTER(exitsensor, NULL, "Stop sensor thread", cmd_exit_sensor);
