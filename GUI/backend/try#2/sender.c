#include <stdio.h>

int main(){

    FILE *fp;

    fp = fopen("data.txt", "w");

    fprintf(fp, "test");

    fclose(fp);

    return 0;
}