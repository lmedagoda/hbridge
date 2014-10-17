#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
int fd_socket;
int fd_left;
socklen_t client_socketlen;
struct sockaddr_in serv_addr, cli_addr;
void start_dagon(){
    fd_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_socket < 0){
        printf("Unable to open Socket\n");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(5001);
    if (bind(fd_socket, (struct sockaddr *) &serv_addr,
                sizeof(serv_addr)) < 0){ 
        printf("Error by binding TCP Socket. Port already used?\n");
    }
    printf("Waiting for TCP from left...\n");
    listen(fd_socket,5);
    client_socketlen = sizeof(cli_addr);
    fd_left = accept(fd_socket, 
            (struct sockaddr *) &cli_addr, 
            &client_socketlen);
    if (fd_left < 0){ 
        printf("Error by accepting the TCP Connection from left %i\n", fd_left);
    }
    printf("TCP Connection etablished succesfully");
}
void run_dagon(){
    if (fd_left < 0){
//        printf("Communication to left aboted...\n");
    }

}
