#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>


int main(int argc, char **argv)
{
    int fd;
    fd = open(argv[1], O_RDWR);
    printf("fd : : %d", fd);
    pwrite(fd, argv[2], 1, 111 );
    close(fd);
    printf("Done\n");
    return(0);
}
