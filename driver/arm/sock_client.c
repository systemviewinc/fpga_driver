#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

void error(char *msg)
{
	perror(msg);
	exit(0);
}

int main(int argc, char *argv[])
{
	int sockfd, portno, n;
	
	struct sockaddr_in serv_addr;
	struct hostent *server;
	
	char buffer[256];
	if (argc < 3) {
		fprintf(stderr,"usage %s hostname port\n", argv[0]);
		exit(0);
	}
	portno = atoi(argv[2]);
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
		error("ERROR opening socket");
	server = gethostbyname(argv[1]);
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		exit(0);
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, 
	      (char *)&serv_addr.sin_addr.s_addr,
	      server->h_length);
	serv_addr.sin_port = htons(portno);
	if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) 
		error("ERROR connecting");
	while (1) {
		printf("Please enter the command: ");
		 for (n = 0 ; n < 256; n++) {
				buffer[n] = n;
		 }	 
		fgets(buffer,255,stdin);		
		n = write(sockfd,buffer,256);
		if (n < 0) error("ERROR writing to socket\n");
		if (strncmp(buffer,"exit",4) == 0) {
			close(sockfd);
			exit(0);
		} else {
			printf("Wrote %d bytes\n",n);
		}
		n = read(sockfd,buffer,256);
		printf("Got back %d bytes %s\n", n, buffer);
	}
	return 0;
}
