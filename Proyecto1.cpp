#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

// Custom function to convert vector<int> to a string
string vectorToString(const vector<int>& vec) {
    ostringstream oss;
    for (const int& num : vec) {
        oss << num << " ";
    }
    return oss.str();
}

// Utilidad para obtener la posición de una ficha en el estado actual
pair<int, int> getBlankPosition(const vector<int>& state) {
    int index = distance(state.begin(), find(state.begin(), state.end(), 0));
    return {index / 4, index % 4};
}

pair<int, int> getInvertedBlankPosition(const vector<int>& state) {
    int index = distance(state.begin(), find(state.begin(), state.end(), 0));
    int row = index / 4;
    int col = index % 4;
    return {3 - row, col}; // Contar las filas desde abajo hacia arriba
}

// Obtener los vecinos para un estado dado
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

// Heurística Walking Distance
int walkingDistance(const vector<int>& state, const vector<int>& goal) {
    int distance = 0;
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
float idaSearch(vector<vector<int>>& path, unordered_set<string>& visited, float g, const vector<int>& current, const vector<int>& goal, float threshold, int& statesGenerated) {
    float f = g + hybridHeuristic(current, goal);
    if (f > threshold) return f;
    if (current == goal) return -1;

    float min = numeric_limits<float>::infinity();
    // Expandir nodos (generar movimientos posibles)
    vector<vector<int>> neighbors = getNeighbors(current);
    for (const auto& neighbor : neighbors) {
        string stateStr = vectorToString(neighbor);
        if (visited.find(stateStr) == visited.end()) { // Evitar ciclos
            visited.insert(stateStr);
            path.push_back(current);
            statesGenerated++;
            float temp = idaSearch(path, visited, g + 1, neighbor, goal, threshold, statesGenerated);
            if (temp == -1) return -1;
            if (temp < min) min = temp;
            path.pop_back();
            visited.erase(stateStr);
        }
    }
    return min;
}

vector<vector<int>> idaStar(const vector<int>& start, const vector<int>& goal, int& statesGenerated) {
    vector<vector<int>> path;
    unordered_set<string> visited;
    float threshold = hybridHeuristic(start, goal);
    statesGenerated = 0;
    while (true) {
        float temp = idaSearch(path, visited, 0, start, goal, threshold, statesGenerated);
        if (temp == -1) {
            path.push_back(goal); // Agregar el estado final
            return path;
        }
        if (temp == numeric_limits<float>::infinity()) return {}; // No se encontró camino
        threshold = temp;
    }
}

vector<int> parseInput(const string& input) {
    vector<int> result;
    istringstream stream(input);
    int number;
    while (stream >> number) {
        result.push_back(number);
    }
    return result;
}

// Función para contar el número de inversiones en un vector
int countInversions(const vector<int>& state) {
    int inversions = 0;
    for (size_t i = 0; i < state.size(); ++i) {
        for (size_t j = i + 1; j < state.size(); ++j) {
            if (state[i] && state[j] && state[i] > state[j]) {
                ++inversions;
            }
        }
    }
    return inversions;
}

// Función para determinar si un rompecabezas es resoluble
bool isSolvable(const vector<int>& state) {
    int inversions = countInversions(state);
    int blankRow = getInvertedBlankPosition(state).first; // Obtener la fila del espacio en blanco
    // Para un rompecabezas de tamaño 4x4:
    // Si la suma de las inversiones y la fila del espacio en blanco es par, el rompecabezas es resoluble
    // Si es impar, no es resoluble
    return (inversions + blankRow) % 2 == 0;
}

// Función principal
int main() {
    string input;
    getline(cin, input); // Leer la entrada estándar
    vector<int> start = parseInput(input);
    
    vector<int> goal = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0};
    if (!isSolvable(start)) {
        cout << "No hay solución." << endl;
        return 0;
    }

    int statesGenerated;
    vector<vector<int>> path = idaStar(start, goal, statesGenerated);

    for (const auto& state : path) {
        for (int i = 0; i < 16; ++i) {
            cout << state[i] << " ";
            if ((i + 1) % 4 == 0) cout << endl;
        }
        cout << endl; 
    }
    cout << "Longitud del camino: " << path.size() - 1 << endl;
    cout << "Número de estados generados: " << statesGenerated << endl;

    return 0;
}