#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <signal.h>
#include <sys/wait.h>

int setup_tier2(void)
{
size_t buffer_size = 10000000000;
char *file_shared_buf;
int ptr;
char buf[20];
size_t nbytes;

ptr = open("obj.txt", O_RDONLY | O_CREAT | O_TRUNC);
//file_shared_buf = mmap(NULL, buffer_size, PROT_READ, MAP_PRIVATE|MAP_FIXED|MAP_DENYWRITE, -1, 0);
mmap(0x6042000, buffer_size, PROT_READ, MAP_SHARED|MAP_FIXED|MAP_DENYWRITE, 4, 0x0000);
strcpy(buf, "This is a test\n");
nbytes = strlen(buf);
write(ptr, buf, nbytes);
return(0);
}


