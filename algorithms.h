#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <stdbool.h>

struct Node {
    int row, col;
};

struct Cell {
    int row, col;
    int parentRow, parentCol;
    int f, g, h;
};

int prim(int** maze, int mazeDimention);

int Astar(int** maze, int mazeDimention, int* start, int* goal);

int calculateHeuristic(int row, int col, int* goal);

bool isValidCell(int row, int col, int mazeDimention);

void addToOpenList(struct Cell* openList, int openListCount, int row, int col, int fChange);

void addPathToMaze(int** maze, struct Cell** cellDetails, int* goal);

#endif