// LD_PRELOAD shim: rewrite INADDR_ANY (0.0.0.0) bind() to the IP in $XT_BIND_IP.
// Lets two XT-M60 SDK processes bind the same UDP port on different host IPs,
// so the kernel routes each radar's stream to the correct socket.
//
// Build:  gcc -shared -fPIC -o xt_bindshim.so xt_bindshim.c -ldl
// Use:    XT_BIND_IP=192.168.0.100 XT_BIND_PORT=7687 \
//         LD_PRELOAD=$PWD/scripts/xt_bindshim.so <command>
// Note:   Optional/manual. The default dual-radar path uses the SDK setUdpDestIp
//         (distinct ports 7687/7688), so this shim is not wired into any launch.
//         The compiled .so is intentionally NOT committed; rebuild from this source.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>

typedef int (*bind_fn)(int, const struct sockaddr *, socklen_t);

int bind(int fd, const struct sockaddr *addr, socklen_t len) {
    static bind_fn real = NULL;
    if (!real) real = (bind_fn)dlsym(RTLD_NEXT, "bind");
    const char *ip = getenv("XT_BIND_IP");
    const char *port_env = getenv("XT_BIND_PORT");
    unsigned short want = (unsigned short)(port_env && *port_env ? atoi(port_env) : 7687);
    if (ip && *ip && addr && addr->sa_family == AF_INET &&
        len >= (socklen_t)sizeof(struct sockaddr_in)) {
        struct sockaddr_in sa;
        memcpy(&sa, addr, sizeof(sa));
        // Only pin the radar data port; leave DDS/other sockets untouched.
        if (sa.sin_addr.s_addr == INADDR_ANY && ntohs(sa.sin_port) == want) {
            sa.sin_addr.s_addr = inet_addr(ip);
            return real(fd, (const struct sockaddr *)&sa, sizeof(sa));
        }
    }
    return real(fd, addr, len);
}
