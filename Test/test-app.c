#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <arpa/inet.h>

#define LORA_MTU 255

int main(int argc, char *argv[])
{
    int sockfd;
    struct sockaddr_ll sk_addr;
    struct ifreq ifr;
    char buffer[LORA_MTU];

    if (argc < 2) {
        puts("Wrong Format! (Usage: ./chat-app <interface>)");
        return -1;
    }
    sockfd = socket(AF_PACKET, SOCK_RAW, 0);
    if (sockfd == -1) {
        perror("socket");
        return -1;
    }
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, argv[1], IFNAMSIZ);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        close(sockfd);
        return -1;
    } 

    memset(&sk_addr, 0, sizeof(sk_addr));
    sk_addr.sll_ifindex = ifr.ifr_ifindex;
    sk_addr.sll_protocol = 0;

    printf("Connected to interface %s. Type messages to send:\n", argv[1]);

    while (1) {
        printf(">> ");
        fflush(stdout);

        if (!fgets(buffer, LORA_MTU, stdin)) {
            perror("fgets");
            break;
        }
        buffer[strcspn(buffer, "\n")] = '\0';

        if (strlen(buffer) == 0) {
            continue;
        }

        if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&sk_addr, sizeof(sk_addr)) < 0) {
            perror("sendto");
            break;
        }
    }
    close(sockfd);
    return 0;
}