#include <algorithm>
#include <iostream>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

// Utilidad para obtener la posición de una ficha en el estado actual
pair<int, int> getBlankPosition(const vector<int>& state) {
    int index = distance(state.begin(), find(state.begin(), state.end(), 0));
    return {index / 4, index % 4};
}

// Generar vecinos para un estado dado
vector<vector<int>> getNeighbors(const vector<int>& state) {
    vector<vector<int>> neighbors;
    auto [blankRow, blankCol] = getBlankPosition(state);

    // Movimientos posibles: arriba, abajo, izquierda, derecha
    vector<pair<int, int>> moves = {
        {blankRow - 1, blankCol}, // Arriba
        {blankRow + 1, blankCol}, // Abajo
        {blankRow, blankCol - 1}, // Izquierda
        {blankRow, blankCol + 1}  // Derecha
    };

    for (const auto& move : moves) {
        int newRow = move.first;
        int newCol = move.second;
        if (newRow >= 0 && newRow < 4 && newCol >= 0 && newCol < 4) {
            vector<int> newState = state;
            swap(newState[blankRow * 4 + blankCol], newState[newRow * 4 + newCol]);
            neighbors.push_back(newState);
        }
    }

    return neighbors;
}

// Utilidad para obtener la posición de una ficha en el estado actual
pair<int, int> getPosition(const vector<int>& state, int value) {
    int index = distance(state.begin(), find(state.begin(), state.end(), value));
    return {index / 4, index % 4};
}

// Heurística de Distancia Manhattan
int manhattanDistance(const vector<int>& state, const vector<int>& goal) {
    int distance = 0;
    for (int i = 0; i < 16; ++i) {
        if (state[i] != 0) {
            auto [goalRow, goalCol] = getPosition(goal, state[i]);
            distance += abs(goalRow - i / 4) + abs(goalCol - i % 4);
        }
    }
    return distance;
}
int countLinearConflicts(const vector<int>& stateLine, const vector<int>& goalLine);

// Heurística de Conflicto Lineal
int linearConflict(const vector<int>& state, const vector<int>& goal) {
    int conflict = 0;
    // Chequeo de conflictos en filas y columnas
    for (int i = 0; i < 4; ++i) {
        // Filas
        vector<int> stateRow(4), goalRow(4);
        for (int j = 0; j < 4; ++j) {
            stateRow[j] = state[i * 4 + j];
            goalRow[j] = goal[i * 4 + j];
        }
        conflict += countLinearConflicts(stateRow, goalRow);
        
        // Columnas
        vector<int> stateCol(4), goalCol(4);
        for (int j = 0; j < 4; ++j) {
            stateCol[j] = state[j * 4 + i];
            goalCol[j] = goal[j * 4 + i];
        }
        conflict += countLinearConflicts(stateCol, goalCol);
    }
    return conflict;
}

int countLinearConflicts(const vector<int>& stateLine, const vector<int>& goalLine) {
    int conflict = 0;
    for (int i = 0; i < 4; ++i) {
        if (stateLine[i] != 0 && stateLine[i] != goalLine[i]) {
            for (int j = i + 1; j < 4; ++j) {
                if (stateLine[j] != 0 && stateLine[j] != goalLine[j] && goalLine[i] == stateLine[j] && goalLine[j] == stateLine[i]) {
                    conflict += 2;
                }
            }
        }
    }
    return conflict;
}

// Heurística de Distancia de Caminata (Walking Distance)
int walkingDistance(const vector<int>& state, const vector<int>& goal) {
    int distance = 0;
    // Suponemos que es similar a la distancia Manhattan para este ejemplo
    for (int i = 0; i < 16; ++i) {
        if (state[i] != 0) {
            auto [goalRow, goalCol] = getPosition(goal, state[i]);
            distance += abs(goalRow - i / 4) + abs(goalCol - i % 4);
        }
    }
    return distance;
}

// Heurística Combinada HH
int hybridHeuristic(const vector<int>& state, const vector<int>& goal) {
    int md = manhattanDistance(state, goal);
    int lc = linearConflict(state, goal);
    int wd = walkingDistance(state, goal);
    return (md / 3) + lc + wd;
}

// IDA* con HH
float idaSearch(vector<vector<int>>& path, float g, const vector<int>& current, const vector<int>& goal, float threshold) {
    float f = g + hybridHeuristic(current, goal);
    if (f > threshold) return f;
    if (current == goal) return -1;

    float min = numeric_limits<float>::infinity();
    // Expandir nodos (generar movimientos posibles)
    vector<vector<int>> neighbors = getNeighbors(current);
    for (const auto& neighbor : neighbors) {
        if (find(path.begin(), path.end(), neighbor) == path.end()) { // Evitar ciclos
            path.push_back(current);
            float temp = idaSearch(path, g + 1, neighbor, goal, threshold);
            if (temp == -1) return -1;
            if (temp < min) min = temp;
            path.pop_back();
        }
    }
    return min;
}

vector<vector<int>> idaStar(const vector<int>& start, const vector<int>& goal) {
    vector<vector<int>> path;
    float threshold = hybridHeuristic(start, goal);
    while (true) {
        float temp = idaSearch(path, 0, start, goal, threshold);
        if (temp == -1) return path;
        if (temp == numeric_limits<float>::infinity()) return {}; // No se encontró camino
        threshold = temp;
    }
}

// Función principal de ejemplo
int main() {
    vector<int> start = {
        1, 4, 2, 3,
        11, 6, 7, 8,
        9, 13, 5, 12,
        0, 14, 15, 10
    };
    
    vector<int> goal = {
        1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12,
        13, 14, 15, 0
    };
    
    vector<vector<int>> path = idaStar(start, goal);
    cout << "Path length: " << path.size() << endl;
    return 0;
}
