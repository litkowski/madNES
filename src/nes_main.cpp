#include <stdio.h>

#include "read.h"

int main(int argc, char* argv[]){

    if (argc == 0) {
        printf("ERROR, no arguments provided.\n"
            "Provide a path to a .nes file\n");
        return 0;
    }

    FILE* game = fopen(argv[1], "r+");

    if (game == NULL) {
        printf("ERROR, file %s does not exist in current context.\n", argv[0]);
    }

    read_header(game);

}
