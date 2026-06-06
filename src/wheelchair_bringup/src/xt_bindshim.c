#define _GNU_SOURCE

#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>

typedef int (*bind_fn)(int, const struct sockaddr *, socklen_t);

int bind(int fd, const struct sockaddr *addr, socklen_t len)
{
  static bind_fn real_bind = NULL;
  const char *ip;
  const char *port_env;
  unsigned short port;

  if (real_bind == NULL) {
    real_bind = (bind_fn)dlsym(RTLD_NEXT, "bind");
  }

  ip = getenv("XT_BIND_IP");
  port_env = getenv("XT_BIND_PORT");
  port = (unsigned short)((port_env != NULL && *port_env != '\0') ? atoi(port_env) : 7687);
  if (ip != NULL && *ip != '\0' && addr != NULL && addr->sa_family == AF_INET &&
      len >= (socklen_t)sizeof(struct sockaddr_in)) {
    struct sockaddr_in pinned;
    memcpy(&pinned, addr, sizeof(pinned));
    if (pinned.sin_addr.s_addr == INADDR_ANY && ntohs(pinned.sin_port) == port) {
      pinned.sin_addr.s_addr = inet_addr(ip);
      return real_bind(fd, (const struct sockaddr *)&pinned, sizeof(pinned));
    }
  }

  return real_bind(fd, addr, len);
}
