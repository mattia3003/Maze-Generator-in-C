#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "algorithms.h"

int main(int argc, char const *argv[])
{
    srand(time(NULL));

    int mazeDimention = 0;
    int result = 0;
    bool validInput = false;
    char junkChars[256];
    

    // Take user input and verify number bigger than 3
    // NB! This validation is not perfect and has edgecase issues, but due to program simplicity, it assumes good intent from user
    while (!validInput) {
        printf("Type desired maze dimention (whole number atleast bigger than 3): ");
        result = scanf("%d", &mazeDimention);

        if (result == 1 && mazeDimention <= 3) {
            printf("Please type number bigger than 3\n");
            continue;
        }

        if (result == 0) {
            printf("invalid input, please type a whole number bigger than 3\n");
            scanf("%s", junkChars);
        }

        if (result != 0 && mazeDimention >= 4) {
            printf("Chosen dimention is: %d \n", mazeDimention);
            validInput = true;
        }
    }

    // Dynamically allocate memory (double pointer) for an array of pointers, each representing a row in the maze
    int **pMazeDimention = (int **) malloc(mazeDimention * sizeof(int *));
    // Check if memory allocation failed
    if (pMazeDimention == NULL) {
        printf("Memory allocation for rows failed. \n");
        return 1; // Indicate error
    }

    // Dynamically allocate memory for each row (Single pointers)
    for (int i = 0; i < mazeDimention; i++) {
        pMazeDimention[i] = (int *) malloc(mazeDimention * sizeof(int));
        // Check if memory allocation failed
        if (pMazeDimention[i] == NULL) {
            printf("Memory allocation for colums %d failed.\n", i);
            // Clean up used memory before exiting with error
            for (int j = 0; j < i; j++) {
                free(pMazeDimention[j]);
            }
            free(pMazeDimention);
            return 1; // Indicate error
        }
    }

    // Set entire maze to 1
    for (int i = 0; i < mazeDimention; i++) {
        for (int j = 0; j < mazeDimention; j++) {
            pMazeDimention[i][j] = 1;
        }
    }

    printf("before prim\n");

    // Create int to capture maze gen return
    int primReturn = 0;
    primReturn = prim(pMazeDimention, mazeDimention);
    // Stop program if return is 1, as maze gen failed
    if (primReturn == 1) {
        printf("Maze generation with prim's algorithm failed\n");
        return 1;
    }

    // Show current maze after maze generation
    for (int i = 0; i < mazeDimention; i++) {
        for (int j = 0; j < mazeDimention; j++) {
            printf("%d  ", pMazeDimention[i][j]);
        }
        printf("\n");
    }

    // Select start and goal (try to set start top left, try to set goal bottom right)
    int start[2] = {0, 0};
    int goal[2] = {0, mazeDimention - 1};

    for (int i = mazeDimention - 1; i > -1; i--) {
        if (pMazeDimention[i][0] == 0) {
            start[0] = i;
            break;
        }
    }

    for (int i = 0; i < mazeDimention; i++) {
        if (pMazeDimention[i][mazeDimention - 1] == 0) {
            goal[0] = i;
            break;
        }
    }

    printf("Start: %d:%d\n", start[0], start[1]);
    printf("Goal: %d:%d\n", goal[0], goal[1]);

    printf("before astar\n");

    int astarReturn = 0;
    astarReturn = Astar(pMazeDimention, mazeDimention, start, goal);
    if (astarReturn == 1) {
        printf("Astar pathfinding failed\n");
        return 1;
    }

    // Show current maze after astar
    for (int i = 0; i < mazeDimention; i++) {
        for (int j = 0; j < mazeDimention; j++) {
            printf("%d  ", pMazeDimention[i][j]);
        }
        printf("\n");
    }

    // Free memory
    for (int i = 0; i < mazeDimention; i++) {
        free(pMazeDimention[i]);
    }
    free(pMazeDimention);

    printf("Entire main ran\n");
    return 0;
}