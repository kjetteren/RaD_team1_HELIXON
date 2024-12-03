#include <stdio.h>
#include <stdlib.h>

int main() {
    FILE *fp;
    int myInt = 5;

    fp = fopen("output.txt", "w+"); // "w+" means open for reading and writing
    if (fp == NULL) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    int chars_written = fprintf(fp, "This is being written in the file. This is an int variable: %d\n", myInt);

    if (chars_written < 0) {
        perror("Error writing to file");
        fclose(fp);
        exit(EXIT_FAILURE);
    }

    printf("\nSuccessfully printed into a file\n");

    if (fclose(fp) != 0) {
        perror("Error closing file");
        exit(EXIT_FAILURE);
    }

    return 0;
}
