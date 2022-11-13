/***************************************************************************/
/*                                                                         */    
/* Client program which gets as parameter the server name or               */    
/*     address and tries to send the data into non-blocking server.        */    
/*                                                                         */    
/* The message is sent after 5 seconds of wait                             */    
/*                                                                         */    
/*                                                                         */    
/* based on Beej's program - look in the simple TCp client for further doc.*/    
/*                                                                         */    
/*                                                                         */    
/***************************************************************************/
    #include <stdio.h> 
    #include <stdlib.h> 
    #include <errno.h> 
    #include <string.h> 
    #include <netdb.h> 
    #include <sys/types.h> 
    #include <netinet/in.h> 
    #include <sys/socket.h> 
    #include <unistd.h>

    #define PORT 5252    /* the port client will be connecting to */

    #define MAXDATASIZE 2048 /* max number of bytes we can get at once */

    int main(int argc, char *argv[])
    {
        int sockfd, numbytes,TEST_ITERATIONS,i,nread,tread;
        TEST_ITERATIONS = 10;  
        char buf[MAXDATASIZE];
        char *ptrbuf;
        struct hostent *he;
        struct sockaddr_in their_addr; /* connector's address information */

        if (argc != 2) {
            fprintf(stderr,"usage: client hostname\n");
            exit(1);
        }

        if ((he=gethostbyname(argv[1])) == NULL) {  /* get the host info */
            herror("gethostbyname");
            exit(1);
        }

        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            perror("socket");
            exit(1);
        }

        their_addr.sin_family = AF_INET;      /* host byte order */
        their_addr.sin_port = htons(PORT);    /* short, network byte order */
        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
        bzero(&(their_addr.sin_zero), 8);     /* zero the rest of the struct */

        if (connect(sockfd, (struct sockaddr *)&their_addr, \
                                              sizeof(struct sockaddr)) == -1) {
            perror("connect");
            exit(1);
        }
    for(i=0;i<10;i++) {
        tread=0;
        ptrbuf=&buf[0];
        while(tread < MAXDATASIZE) {
            nread=read(sockfd, ptrbuf, MAXDATASIZE);
            printf("%d %d\n",nread,tread);
            if(tread != MAXDATASIZE) {
                 
                ptrbuf=&buf[0] + nread;
                nread=read(sockfd, ptrbuf, MAXDATASIZE);
                tread=tread+nread;
                printf("%d %d\n",nread,tread);
                tread=2049;
                 
            }
             
 
             
        }
    //buf[2048]='\0';    
    printf("%d\n",buf[2048]);        
    //printf("%s\n",buf);
    printf("After the read function %d \n",i);
    
    if (send(sockfd, buf, MAXDATASIZE,0) == -1){
                      perror("send");
		      exit (1);
    }
    
        
 
    sleep(10);

	}

        close(sockfd);

        return 0;
    }


