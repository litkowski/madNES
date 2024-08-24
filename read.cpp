#include <stdio.h>
#include <stdlib.h>

// Read the header of the game ROM and print its contents
int read_header(FILE* game){

    char* header = malloc(sizeof(char) * 17);
    header[16] = '\0';
    fread(header, 1, 16, game);

    for (int i = 0; i <= 16; i++) {
        printf("%c", header[i]);
    }

    printf("\n");

    printf("PRG ROM size: %d KB\n", 16 * header[4]);
    printf("CHR ROM size: %d KB\n", 8 * header[5]);

    free(header);

}

int read_ROM(){

}
