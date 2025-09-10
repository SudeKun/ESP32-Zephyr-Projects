#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/wifi_mgmt.h>
#include <string.h>
#include <errno.h>

static void print_udp_smp_addr(struct net_if *iface)
{
#ifdef CONFIG_MCUMGR_TRANSPORT_UDP
    if (!iface) {
        return;
    }

    struct net_if_config *cfg = net_if_get_config(iface);
    if (cfg && cfg->ip.ipv4) {
        struct net_if_addr_ipv4 *ua = &cfg->ip.ipv4->unicast[0];
        if (ua->ipv4.addr_state == NET_ADDR_TENTATIVE ||
            ua->ipv4.addr_state == NET_ADDR_PREFERRED) {
            char addr[NET_IPV4_ADDR_LEN];
            net_addr_ntop(AF_INET, &ua->ipv4.address.in_addr, addr, sizeof(addr));
            printk("UDP SMP: %s:%d\n", addr, CONFIG_MCUMGR_TRANSPORT_UDP_PORT);
        }
    }
#endif
}

static struct net_mgmt_event_callback ipv4_cb;
static bool cb_registered;

static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
                               uint32_t mgmt_event, struct net_if *iface)
{
    ARG_UNUSED(cb);

    if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD ||
        mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
        print_udp_smp_addr(iface);
    }
}

// Called from main() to print where UDP SMP will listen
int start_smp_udp(void)
{
#ifdef CONFIG_MCUMGR_TRANSPORT_UDP
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        printk("UDP SMP: no default net_if\n");
        return -ENODEV;
    }

    // Try to print immediately if IPv4 is already assigned
    struct net_if_config *cfg = net_if_get_config(iface);
    if (cfg && cfg->ip.ipv4) {
        struct net_if_addr_ipv4 *ua = &cfg->ip.ipv4->unicast[0];
        if (ua->ipv4.addr_state == NET_ADDR_TENTATIVE ||
            ua->ipv4.addr_state == NET_ADDR_PREFERRED) {
                print_udp_smp_addr(iface);
                return 0;
        }
    }

    // Otherwise wait for IPv4 via net mgmt events
    if (!cb_registered) {
        net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler,
                                     NET_EVENT_IPV4_ADDR_ADD |
                                     NET_EVENT_IPV4_DHCP_BOUND);
        net_mgmt_add_event_callback(&ipv4_cb);
        cb_registered = true;
    }
    printk("UDP SMP: waiting for IPv4\n");
#else
    printk("UDP SMP transport not enabled\n");
#endif
    return 0;
}