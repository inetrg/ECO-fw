/*
 * based on examples/gcoap/gcoap_cli.c
 */

 #include <stdint.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "net/gcoap.h"
 #include "od.h"
 #include "fmt.h"
 #define LOG_LEVEL LOG_NONE
 #include "log.h"

mutex_t mutex_blocking_req = MUTEX_INIT;
kernel_pid_t blocking_pid = KERNEL_PID_UNDEF;

 static void _resp_handler(unsigned req_state, coap_pkt_t* pdu,
                           sock_udp_ep_t *remote);

/*
 * Response callback.
 */
static void _resp_handler(unsigned req_state, coap_pkt_t* pdu,
                          sock_udp_ep_t *remote)
{
    (void)remote;       /* not interested in the source currently */

    if (blocking_pid != KERNEL_PID_UNDEF) {
        msg_t m;
        m.content.value = coap_get_code_class(pdu) << 8 | coap_get_code_detail(pdu);
        msg_send(&m,blocking_pid);
    }

    if (req_state == GCOAP_MEMO_TIMEOUT) {
        LOG_DEBUG("gcoap: timeout for msg ID %02u\n", coap_get_id(pdu));
        return;
    }
    else if (req_state == GCOAP_MEMO_ERR) {
        LOG_DEBUG("gcoap: error in response\n");
        return;
    }

    char *class_str = (coap_get_code_class(pdu) == COAP_CLASS_SUCCESS)
                            ? "Success" : "Error";
    LOG_DEBUG("gcoap: response %s, code %1u.%02u", class_str,
                                                   coap_get_code_class(pdu),
                                                   coap_get_code_detail(pdu));

    if (pdu->payload_len) {
        if (coap_get_content_type(pdu) == COAP_FORMAT_TEXT
                || coap_get_content_type(pdu) == COAP_FORMAT_LINK
                || coap_get_code_class(pdu) == COAP_CLASS_CLIENT_FAILURE
                || coap_get_code_class(pdu) == COAP_CLASS_SERVER_FAILURE) {
            /* Expecting diagnostic payload in failure cases */
            LOG_DEBUG(", %u bytes\n%.*s\n", pdu->payload_len, pdu->payload_len,
                                                          (char *)pdu->payload);
        }
        else {
            LOG_DEBUG(", %u bytes\n", pdu->payload_len);
            od_hex_dump(pdu->payload, pdu->payload_len, OD_WIDTH_DEFAULT);
        }
    }
    else {
        LOG_DEBUG(", empty payload\n");
    }
}

static size_t _send(uint8_t *buf, size_t len, char *addr_str, char *port_str)
{
    ipv6_addr_t addr;
    sock_udp_ep_t remote;

    remote.family = AF_INET6;

    /* parse for interface */
    int iface = ipv6_addr_split_iface(addr_str);
    if (iface == -1) {
        if (gnrc_netif_numof() == 1) {
            /* assign the single interface found in gnrc_netif_numof() */
            remote.netif = (uint16_t)gnrc_netif_iter(NULL)->pid;
        }
        else {
            remote.netif = SOCK_ADDR_ANY_NETIF;
        }
    }
    else {
        if (gnrc_netif_get_by_pid(iface) == NULL) {
            LOG_DEBUG("gcoap_cli: interface not valid\n");
            return 0;
        }
        remote.netif = iface;
    }

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        LOG_DEBUG("gcoap_cli: unable to parse destination address\n");
        return 0;
    }
    if ((remote.netif == SOCK_ADDR_ANY_NETIF) && ipv6_addr_is_link_local(&addr)) {
        LOG_DEBUG("gcoap_cli: must specify interface for link local target\n");
        return 0;
    }
    memcpy(&remote.addr.ipv6[0], &addr.u8[0], sizeof(addr.u8));

    /* parse port */
    remote.port = atoi(port_str);
    if (remote.port == 0) {
        LOG_DEBUG("gcoap_cli: unable to parse destination port\n");
        return 0;
    }
    LOG_DEBUG("gcoap_req_send2...\n");
    return gcoap_req_send2(buf, len, &remote, _resp_handler);
}

/*
* @param[in]  code      coap request code COAP_{GET,POST,PUT,DELETE}
* @param[in]  addr_str  IPv6 addr as string (may be suffixed with %x iface idx)
* @param[in]  port_str  destination port as string
* @param[in]  path      destination ressource path (must start with /)
* @param[in]  data      optional payload
* @param[in]  len       size of payload
* @param[in]  confirm   true for confirmable message otherwise false
*
* @return  0 on success
* @return  < 0 on error
*/
int coap_request(unsigned code, char *addr_str, char *port_str, char *path, uint8_t *data, size_t len, bool confirm)
{
    uint8_t buf[GCOAP_PDU_BUF_SIZE];
    coap_pkt_t pdu;

    gcoap_req_init(&pdu, &buf[0], GCOAP_PDU_BUF_SIZE, code, path);
    if (len > 0) {
        memcpy(pdu.payload, data, len);
    }

    coap_hdr_set_type(pdu.hdr, confirm == true ? COAP_TYPE_CON : COAP_TYPE_NON);

    if (len > 0) {
        len = gcoap_finish(&pdu, len, COAP_FORMAT_TEXT);
    }
    else {
        len = gcoap_finish(&pdu, 0, COAP_FORMAT_NONE);
    }

    LOG_DEBUG("coap_request: sending msg ID %u, %u bytes to %s\n",
              coap_get_id(&pdu),
              (unsigned) len,
              addr_str);
    if (!_send(&buf[0], len, addr_str, port_str)) {
        LOG_DEBUG("gcoap_cli: msg send failed\n");
    }
    return 0;
}

/*
* @param[in]  code      coap request code COAP_{GET,POST,PUT,DELETE}
* @param[in]  addr_str  IPv6 addr as string (may be suffixed with %x iface idx)
* @param[in]  port_str  destination port as string
* @param[in]  path      destination ressource path (must start with /)
* @param[in]  data      optional payload
* @param[in]  len       size of payload
* @param[in]  timeout   timeout in [ms]
*
* @return  0 on success
* @return  < 0 on error
*/
int coap_request_blocking(unsigned code, char *addr_str, char *port_str,
                          char *path, uint8_t *data, size_t len, int timeout)
{
    mutex_lock(&mutex_blocking_req);
    int res = -1;
    blocking_pid = thread_getpid();
    if (coap_request(code, addr_str, port_str, path, data, len, true) == 0) {
        uint32_t end = xtimer_now_usec() + timeout * 1000;
        msg_t m;
        while(xtimer_now_usec() < end){
            if( msg_try_receive(&m) == 1) {
              LOG_DEBUG("got msg... took %lu us\n", xtimer_now_usec() - (end - timeout * 1000));
              res = 0;
              break;
            };
            xtimer_usleep(10 * 1000);
        }
    }
    blocking_pid = KERNEL_PID_UNDEF;
    mutex_unlock(&mutex_blocking_req);
    if(res != 0){
        LOG_DEBUG("--------------->TIMEOUT\n");
    }
    return res;
}

int coap_post_blocking(char *path, char *addr_str, char *json, size_t json_len){

  uint32_t blocking_reg_start = xtimer_now_usec();

  int coap_res = coap_request_blocking(COAP_POST,
                                       addr_str,
                                       "5683",
                                       path,
                                       (unsigned char*)json,
                                       json_len,
                                       GCOAP_BLOCKING_TIMEOUT_MS);

  LOG_DEBUG("returned from coap_request_blocking after %lu\n", xtimer_now_usec() - blocking_reg_start);

  if (coap_res == 0) {
      LOG_DEBUG("coap request delivered\n");
      return 0;
  }

  LOG_INFO("coap request failed\n");
  return -1;
}
