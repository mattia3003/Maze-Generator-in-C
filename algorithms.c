#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>
#include "algorithms.h"

// Structure to maintain positional information about spesific "nodes" for prim maze generation
struct Node {
    int row, col;
};



// Structure to maintain positional information and cost details about spesific "cells" for astar path finding
struct Cell {
    int row, col;
    int parentRow, parentCol;
    int f, g, h;
};



// Define prototypes
int calculateHeuristic(int row, int col, int* goal);

bool isValidCell(int row, int col, int mazeDimention);

void addToOpenList(struct Cell* openList, int openListCount, int row, int col, int fChange);

void addPathToMaze(int** maze, struct Cell** cellDetails, int* goal);



// Function for maze generation with a modified verison of Prim's algorithm for creation of minimum spanning tree, from a weighted undirected graph structure
int prim(int** maze, int mazeDimention) {

    //INFO ABOUT MAZE: 0 = path, 1 = wall, -1 = unexplored wall

    // Set random start point
    int randRowStart = rand() % mazeDimention;
    int randColStart = rand() % mazeDimention;
    maze[randRowStart][randColStart] = 0;

    int wallCount = 0;

    // Pointer to array of structs, the same size as the number of nodes in the maze, each struct with two int's, one for row coords and one for col coords
    // The wallCount will keep the index of the "last" struct in the array being used, and the rest of the structs after the index are open to be filled at demand
    struct Node *pUnexploredWalls = (struct Node *) calloc(mazeDimention * mazeDimention, sizeof(struct Node));
    
    // Check if memory allocation failed
    if (pUnexploredWalls == NULL) {
        printf("Memory allocation for node struct array failed. \n");
        return 1; // Indicate error
    }

    // Check if surrounding node coordinates (top, bot, left, right) are out of bounds, and add to list of unchecked walls if not out of bounds
    if (randRowStart > 0) {
        // Check top
        maze[randRowStart - 1][randColStart] = -1;
        pUnexploredWalls[wallCount].row = randRowStart - 1;
        pUnexploredWalls[wallCount].col = randColStart;
        wallCount++;
    }
    if (randRowStart < mazeDimention - 1) {
        // Check bot
        maze[randRowStart + 1][randColStart] = -1;
        pUnexploredWalls[wallCount].row = randRowStart + 1;
        pUnexploredWalls[wallCount].col = randColStart;
        wallCount++;
    }
    if (randColStart > 0) {
        // Check left
        maze[randRowStart][randColStart - 1] = -1;
        pUnexploredWalls[wallCount].row = randRowStart;
        pUnexploredWalls[wallCount].col = randColStart - 1;
        wallCount++;
    }
    if (randColStart < mazeDimention - 1) {
        // Check right
        maze[randRowStart][randColStart + 1] = -1;
        pUnexploredWalls[wallCount].row = randRowStart;
        pUnexploredWalls[wallCount].col = randColStart + 1;
        wallCount++;
    } 

    int loopCount = 0;

    // While there are still walls left to be "explored"
    while (wallCount > 0) {

        loopCount++;

        // Define index (random) and coords for the next wall to be explored
        int nextWallIndex = rand() % wallCount;
        int nextWallRow = pUnexploredWalls[nextWallIndex].row;
        int nextWallCol = pUnexploredWalls[nextWallIndex].col;

        int surrPaths = 0;

        // Check how many "paths'" (0) the wall is in contact with
        if (nextWallRow > 0) {
            // Check top
            if (maze[nextWallRow - 1][nextWallCol] == 0) {
                surrPaths++;
            }
        }

        if (nextWallRow < mazeDimention - 1) {
            // Check bot
            if (maze[nextWallRow + 1][nextWallCol] == 0) {
                surrPaths++;
            }
        }

        if (nextWallCol > 0) {
            // Check left
            if (maze[nextWallRow][nextWallCol - 1] == 0) {
                surrPaths++;
            }
        }

        if (nextWallCol < mazeDimention - 1) {
            // Check right
            if (maze[nextWallRow][nextWallCol + 1] == 0) {
                surrPaths++;
            }
        }

        // If wall is in contact with more than 1, it means it cannot become path, but is set as wall
        if (surrPaths > 1) {
            // Update unexplored wall array, by removing the current explored wall and moving the elements with bigger index one spot to the left
            for (int i = nextWallIndex; i < wallCount; i++) {
                pUnexploredWalls[i].row = pUnexploredWalls[i + 1].row;
                pUnexploredWalls[i].col = pUnexploredWalls[i + 1].col;
            }
            // Set wall, reduse unexplored wall count and exit out of this iteration
            maze[nextWallRow][nextWallCol] = 1;
            wallCount--;
            continue;
        }

        // If wall was not in contact with more than one path (0), loop over again and add wall node neighbours to "unexplored walls"
        if (nextWallRow > 0) {
            // Check top
            if (maze[nextWallRow - 1][nextWallCol] == 1) {
                maze[nextWallRow - 1][nextWallCol] = -1;
                pUnexploredWalls[wallCount].row = nextWallRow - 1;
                pUnexploredWalls[wallCount].col = nextWallCol;
                wallCount++;
            }
        }

        if (nextWallRow < mazeDimention - 1) {
            // Check bot
            if (maze[nextWallRow + 1][nextWallCol] == 1) {
                maze[nextWallRow + 1][nextWallCol] = -1;
                pUnexploredWalls[wallCount].row = nextWallRow + 1;
                pUnexploredWalls[wallCount].col = nextWallCol;
                wallCount++;
            }
        }

        if (nextWallCol > 0) {
            // Check left
            if (maze[nextWallRow][nextWallCol - 1] == 1) {
                maze[nextWallRow][nextWallCol - 1] = -1;
                pUnexploredWalls[wallCount].row = nextWallRow;
                pUnexploredWalls[wallCount].col = nextWallCol - 1;
                wallCount++;
            }
        }

        if (nextWallRow < mazeDimention - 1) {
            // Check right
            if (maze[nextWallRow][nextWallCol + 1] == 1) {
                maze[nextWallRow][nextWallCol + 1] = -1;
                pUnexploredWalls[wallCount].row = nextWallRow;
                pUnexploredWalls[wallCount].col = nextWallCol + 1;
                wallCount++;
            }
        }

        // Update unexplored walls array by removing currently explored wall, and moving elements with bigger index one spot to the left
        for (int i = nextWallIndex; i < wallCount; i++) {
            pUnexploredWalls[i].row = pUnexploredWalls[i + 1].row;
            pUnexploredWalls[i].col = pUnexploredWalls[i + 1].col;
        }
        // Set currently explored wall to be path and reduce unexplored wall count
        maze[nextWallRow][nextWallCol] = 0;
        wallCount--;
    }

    printf("number of loops: %d\n.", loopCount);

    // Free unexplored walls pointer after maze generation is done
    free(pUnexploredWalls);

    return 0;
}



// Function for path finding using A-star algorithm, finding shortest path between two cells in the maze, using heuristics.
int Astar(int** maze, int mazeDimention, int* start, int* goal) {

    // INFO ABOUT RESULT: 0 = walkable path, 1 = wall, 2 = Chosen path

    // Dynamically allocate memory (double pointer) for an array of pointers, each representing a row in the maze
    struct Cell **pCellDetails = (struct Cell **) malloc(mazeDimention * sizeof(struct Cell *));
    // Check if memory allocation failed
    if (pCellDetails == NULL) {
        printf("Memory allocation for rows failed. \n");
        return 1; // Indicate error
    }

    // Dynamically allocate memory for each row (Single pointers)
    for (int i = 0; i < mazeDimention; i++) {
        pCellDetails[i] = (struct Cell *) malloc(mazeDimention * sizeof(struct Cell));
        // Check if memory allocation failed
        if (pCellDetails[i] == NULL) {
            printf("Memory allocation for colums %d failed.\n", i);
            // Clean up used memory before exiting with error
            for (int j = 0; j < i; j++) {
                free(pCellDetails[j]);
            }
            free(pCellDetails);
            return 1; // Indicate error
        }
    }

    // Dynamically allocate memory (double pointer) for an array of pointers, each representing a row in the maze
    bool **pClosedList = (bool **) malloc(mazeDimention * sizeof(bool *));
    // Check if memory allocation failed
    if (pClosedList == NULL) {
        printf("Memory allocation for rows failed. \n");
        return 1; // Indicate error
    }

    // Dynamically allocate memory for each row (Single pointers)
    for (int i = 0; i < mazeDimention; i++) {
        pClosedList[i] = (bool *) malloc(mazeDimention * sizeof(bool));
        // Check if memory allocation failed
        if (pClosedList[i] == NULL) {
            printf("Memory allocation for colums %d failed.\n", i);
            // Clean up used memory before exiting with error
            for (int j = 0; j < i; j++) {
                free(pClosedList[j]);
            }
            free(pClosedList);
            return 1; // Indicate error
        }
    }

    // Set values in closed list to false (indicating that the spesific cell is not in the list), and set values in cell details to start values
    for (int i = 0; i < mazeDimention; i++) {
        for (int j = 0; j < mazeDimention; j++) {
            pClosedList[i][j] = false;

            pCellDetails[i][j].f = INT_MAX;
            pCellDetails[i][j].g = INT_MAX;
            pCellDetails[i][j].h = INT_MAX;
            pCellDetails[i][j].row = i;
            pCellDetails[i][j].col = j;
            pCellDetails[i][j].parentRow = -1;
            pCellDetails[i][j].parentCol = -1;
        }
    }

    // Array of structs holding cells to be explored (These will be sorted based on f value)
    // Although these are cell structs, only the node position and f value are used
    struct Cell *pOpenList = (struct Cell *) calloc(mazeDimention * mazeDimention, sizeof(struct Cell));

    int startRow = start[0];
    int startCol = start[1];

    // Define initial start values for start cell
    pCellDetails[startRow][startCol].f = 0;
    pCellDetails[startRow][startCol].g = 0;
    pCellDetails[startRow][startCol].h = 0;
    pCellDetails[startRow][startCol].parentRow = startRow;
    pCellDetails[startRow][startCol].parentCol = startCol;

    // Add start cell position to open list
    pOpenList[0].row = startRow;
    pOpenList[0].col = startCol;
    int openListCount = 1;

    bool goalReached = false;

    int loopCount = 0;

    // While there are still cells to be explored
    while (openListCount > 0) {

        loopCount++;

        // Select next cell coords to be explored
        int row = pOpenList[0].row;
        int col = pOpenList[0].col;

        // "Remove" cell from open list
        pOpenList[0].f = 0;
        pOpenList[0].row = 0;
        pOpenList[0].col = 0;

        // Move every cell in open list one spot to the left
        for (int i = 0; i < openListCount; i++) {
            pOpenList[i].f = pOpenList[i + 1].f;
            pOpenList[i].row = pOpenList[i + 1].row;
            pOpenList[i].col = pOpenList[i + 1].col;

        }

        openListCount--;

        pClosedList[row][col] = true;

        int fChange, gChange, hChange;

        // Check cell top
        if (isValidCell(row - 1, col, mazeDimention)) {
            // If goal is reached, set current iteration exploring cell as parent of goal cell, add complete path to maze and return
            if ((row - 1) == goal[0] && col == goal[1]) {
                pCellDetails[row - 1][col].parentRow = row;
                pCellDetails[row - 1][col].parentCol = col;
                addPathToMaze(maze, pCellDetails, goal);
                goalReached = true;
                printf("Number of loops: %d\n", loopCount);
                return 0;
            }
            
            // If goal not reached, but the current cell expantion is a valid cell and a path, update cell details
            else if (pClosedList[row - 1][col] == false && maze[row - 1][col] == 0) {
                gChange = pCellDetails[row][col].g + 1;
                hChange = calculateHeuristic(row - 1, col, goal);
                fChange = gChange + hChange;

                if (pCellDetails[row - 1][col].f == INT_MAX || pCellDetails[row - 1][col].f > fChange) {
                    addToOpenList(pOpenList, openListCount, row - 1, col, fChange);
                    openListCount++;
                    pCellDetails[row - 1][col].f = fChange;
                    pCellDetails[row - 1][col].g = gChange;
                    pCellDetails[row - 1][col].h = hChange;
                    pCellDetails[row - 1][col].parentRow = row;
                    pCellDetails[row - 1][col].parentCol = col;
                }
            }
        }

        // Check cell bot
        if (isValidCell(row + 1, col, mazeDimention)) {
            // If goal is reached, set current iteration exploring cell as parent of goal cell, add complete path to maze and return
            if ((row + 1) == goal[0] && col == goal[1]) {
                pCellDetails[row + 1][col].parentRow = row;
                pCellDetails[row + 1][col].parentCol = col;
                addPathToMaze(maze, pCellDetails, goal);
                goalReached = true;
                printf("Number of loops: %d\n", loopCount);
                return 0;
            }
            
            // If goal not reached, but the current cell expantion is a valid cell and a path, update cell details
            else if (pClosedList[row + 1][col] == false && maze[row + 1][col] == 0) {
                gChange = pCellDetails[row][col].g + 1;
                hChange = calculateHeuristic(row + 1, col, goal);
                fChange = gChange + hChange;

                if (pCellDetails[row + 1][col].f == INT_MAX || pCellDetails[row + 1][col].f > fChange) {
                    addToOpenList(pOpenList, openListCount, row + 1, col, fChange);
                    openListCount++;
                    pCellDetails[row + 1][col].f = fChange;
                    pCellDetails[row + 1][col].g = gChange;
                    pCellDetails[row + 1][col].h = hChange;
                    pCellDetails[row + 1][col].parentRow = row;
                    pCellDetails[row + 1][col].parentCol = col;
                }
            }
        }

        // Check cell left
        if (isValidCell(row, col - 1, mazeDimention)) {
            // If goal is reached, set current iteration exploring cell as parent of goal cell, add complete path to maze and return
            if (row == goal[0] && (col - 1) == goal[1]) {
                pCellDetails[row][col - 1].parentRow = row;
                pCellDetails[row][col - 1].parentCol = col;
                addPathToMaze(maze, pCellDetails, goal);
                goalReached = true;
                printf("Number of loops: %d\n", loopCount);
                return 0;
            }
            
            // If goal not reached, but the current cell expantion is a valid cell and a path, update cell details
            else if (pClosedList[row][col - 1] == false && maze[row][col - 1] == 0) {
                gChange = pCellDetails[row][col].g + 1;
                hChange = calculateHeuristic(row, col - 1, goal);
                fChange = gChange + hChange;

                if (pCellDetails[row][col - 1].f == INT_MAX || pCellDetails[row][col - 1].f > fChange) {
                    addToOpenList(pOpenList, openListCount, row, col - 1, fChange);
                    openListCount++;
                    pCellDetails[row][col - 1].f = fChange;
                    pCellDetails[row][col - 1].g = gChange;
                    pCellDetails[row][col - 1].h = hChange;
                    pCellDetails[row][col - 1].parentRow = row;
                    pCellDetails[row][col - 1].parentCol = col;
                }
            }
        }

        // Check cell right
        if (isValidCell(row, col + 1, mazeDimention)) {
            // If goal is reached, set current iteration exploring cell as parent of goal cell, add complete path to maze and return
            if (row == goal[0] && (col + 1) == goal[1]) {
                pCellDetails[row][col + 1].parentRow = row;
                pCellDetails[row][col + 1].parentCol = col;
                addPathToMaze(maze, pCellDetails, goal);
                goalReached = true;
                printf("Number of loops: %d\n", loopCount);
                return 0;
            }
            
            // If goal not reached, but the current cell expantion is a valid cell and a path, update cell details
            else if (pClosedList[row][col + 1] == false && maze[row][col + 1] == 0) {
                gChange = pCellDetails[row][col].g + 1;
                hChange = calculateHeuristic(row, col + 1, goal);
                fChange = gChange + hChange;

                if (pCellDetails[row][col + 1].f == INT_MAX || pCellDetails[row][col + 1].f > fChange) {
                    addToOpenList(pOpenList, openListCount, row, col + 1, fChange);
                    openListCount++;
                    pCellDetails[row][col + 1].f = fChange;
                    pCellDetails[row][col + 1].g = gChange;
                    pCellDetails[row][col + 1].h = hChange;
                    pCellDetails[row][col + 1].parentRow = row;
                    pCellDetails[row][col + 1].parentCol = col;
                }
            }
        }

    }

    if (!goalReached) {
        printf("Failed to reach goal\n");
    }

    return 1;
}

// Utility function to calculate heuristic with Manhattan distance for grids
int calculateHeuristic(int row, int col, int* goal) {
    return abs(row - goal[0]) + abs(col - goal[1]);
}

// Utility function to validate cell coords for astar function
bool isValidCell(int row, int col, int mazeDimention) {
    return (row >= 0) && (row < mazeDimention) && (col >= 0) && (col < mazeDimention);
}

// Utility function to add to and update open list sorted by f value (smallest first)
void addToOpenList(struct Cell* openList, int openListCount, int row, int col, int fChange) {
    int insertIndex = 0;
    // Checks which index to insert new cell
    for (int i = 0; i < openListCount; i++) {
        if (openList[i].f > fChange) {
            break;
        }
        insertIndex++;
    }

    // Moves every cell with bigger f value one slot to the right to make space for the new cell
    for (int i = openListCount; i > insertIndex - 1; i--) {
        openList[i].row = openList[i - 1].row;
        openList[i].col = openList[i - 1].col;
        openList[i].f = openList[i - 1].f;
    }

    // Add new cell to the open slot
    openList[insertIndex].row = row;
    openList[insertIndex].col = col;
    openList[insertIndex].f = fChange;

}

// Utility function for adding the optimal path to the maze, indicated by the number 2
void addPathToMaze(int** maze, struct Cell** cellDetails, int* goal) {
    int row = goal[0];
    int col = goal[1];

    while (!(cellDetails[row][col].parentRow == row && cellDetails[row][col].parentCol == col)) {
        maze[row][col] = 2;
        int nextRow = cellDetails[row][col].parentRow;
        int nextCol = cellDetails[row][col].parentCol;
        row = nextRow;
        col = nextCol;
    }

    maze[row][col] = 2;
}