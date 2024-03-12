#include <stdio.h>

#define PATH "data.txt"

int main(){

    FILE *fp;
     fp = fopen(PATH, "w");

    for(int i=0; i<100; i++){
        fprintf(fp, "%d\n", i);
    }

    fclose(fp);

    return 0;
}