#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int main(void){
    FILE* ptr;
    char ch;
    int i;
    ptr = fopen("secret_number", "r");

    if (NULL == ptr) {
        printf("file can't be opened \n");
        return(-1);
    }
    i = 10000000;
    while(i>0){
        printf("You entered: %d", i);
        printf("\n");
        i--;
    }
        printf("content of this file are \n");

    while (!feof(ptr)) {
        ch = fgetc(ptr);
        printf("%c", ch);
    }
    fclose(ptr);


    return(0);
}
