#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
#include <climits>
#include <unordered_map>
#include <set>

class Attraction {
public:
    std::string name;
    std::map<Attraction*, int> neighbors;
    Attraction(std::string name) : name(name) {}

    void addNeighbor(Attraction* neighbor, int time) {
        neighbors[neighbor] = time;
        neighbor->neighbors[this] = time; // Assuming undirected graph
    }
};

std::pair<std::vector<Attraction*>, int> findPathGreedy(Attraction* start, Attraction* end) {
    std::vector<Attraction*> path;
    std::set<Attraction*> visited;
    Attraction* current = start;
    int totalTime = 0;

    while (current != end) {
        path.push_back(current);
        visited.insert(current);

        int shortestTime = std::numeric_limits<int>::max();
        Attraction* nextAttraction = nullptr;

        for (auto& neighbor : current->neighbors) {
            if (visited.find(neighbor.first) == visited.end() && neighbor.second < shortestTime) {
                shortestTime = neighbor.second;
                nextAttraction = neighbor.first;
            }
        }

        if (nextAttraction == nullptr) {
            return {{}, -1}; // No path found
        }

        current = nextAttraction;
        totalTime += shortestTime;
    }

    path.push_back(end);
    return {path, totalTime};
}

std::pair<std::vector<Attraction*>, int> findShortestPathDijkstra(Attraction* start, Attraction* end) {
    std::map<Attraction*, int> distances;
    std::map<Attraction*, Attraction*> previous;
    std::set<Attraction*> visited;
    std::priority_queue<std::pair<int, Attraction*>, std::vector<std::pair<int, Attraction*>>, std::greater<std::pair<int, Attraction*>>> pq;

    for (auto& neighbor : start->neighbors) {
        distances[neighbor.first] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        Attraction* current = pq.top().second;
        pq.pop();

        if (visited.find(current) != visited.end()) continue;
        visited.insert(current);

        for (auto& neighbor : current->neighbors) {
            int newDist = distances[current] + neighbor.second;
            if (newDist < distances[neighbor.first]) {
                distances[neighbor.first] = newDist;
                previous[neighbor.first] = current;
                pq.push({newDist, neighbor.first});
            }
        }
    }

    std::vector<Attraction*> path;
    if (distances.find(end) == distances.end() || distances[end] == std::numeric_limits<int>::max()) {
        return {{}, -1}; // No path found
    }

    int totalTime = distances[end];
    for (Attraction* at = end; at != nullptr; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    return {path, totalTime};
}

int heuristic(Attraction* a, Attraction* b) {
    return 1; // Replace with a meaningful heuristic if available
}

std::pair<std::vector<Attraction*>, int> findPathAStar(Attraction* start, Attraction* end) {
    std::map<Attraction*, int> gScore;
    std::map<Attraction*, int> fScore;
    std::map<Attraction*, Attraction*> previous;
    std::set<Attraction*> openSet;
    std::priority_queue<std::pair<int, Attraction*>, std::vector<std::pair<int, Attraction*>>, std::greater<std::pair<int, Attraction*>>> pq;

    gScore[start] = 0;
    fScore[start] = heuristic(start, end);
    pq.push({fScore[start], start});
    openSet.insert(start);

    while (!pq.empty()) {
        Attraction* current = pq.top().second;
        pq.pop();

        if (current == end) {
            std::vector<Attraction*> path;
            int totalTime = gScore[end];
            for (Attraction* at = end; at != nullptr; at = previous[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return {path, totalTime};
        }

        openSet.erase(current);

        for (auto& neighbor : current->neighbors) {
            int tentativeGScore = gScore[current] + neighbor.second;
            if (tentativeGScore < gScore[neighbor.first]) {
                previous[neighbor.first] = current;
                gScore[neighbor.first] = tentativeGScore;
                fScore[neighbor.first] = gScore[neighbor.first] + heuristic(neighbor.first, end);
                if (openSet.find(neighbor.first) == openSet.end()) {
                    openSet.insert(neighbor.first);
                    pq.push({fScore[neighbor.first], neighbor.first});
                }
            }
        }
    }

    return {{}, -1}; // No path found
}

std::vector<std::pair<Attraction*, Attraction*>> findMST(std::vector<Attraction*> attractions) {
    std::vector<std::pair<Attraction*, Attraction*>> mst;
    std::set<Attraction*> inMST;
    std::priority_queue<std::pair<int, std::pair<Attraction*, Attraction*>>, std::vector<std::pair<int, std::pair<Attraction*, Attraction*>>>, std::greater<std::pair<int, std::pair<Attraction*, Attraction*>>>> pq;

    Attraction* start = attractions[0];
    inMST.insert(start);

    for (auto& neighbor : start->neighbors) {
        pq.push({neighbor.second, {start, neighbor.first}});
    }

    while (!pq.empty()) {
        auto edge = pq.top();
        pq.pop();

        Attraction* u = edge.second.first;
        Attraction* v = edge.second.second;
        int weight = edge.first;

        if (inMST.find(v) == inMST.end()) {
            mst.push_back({u, v});
            inMST.insert(v);

            for (auto& neighbor : v->neighbors) {
                if (inMST.find(neighbor.first) == inMST.end()) {
                    pq.push({neighbor.second, {v, neighbor.first}});
                }
            }
        }
    }

    return mst;
}

void printPath(const std::vector<Attraction*>& path) {
    for (Attraction* attraction : path) {
        std::cout << attraction->name << " ";
    }
    std::cout << std::endl;
}

int main() {
    Attraction* A = new Attraction("A");
    Attraction* B = new Attraction("B");
    Attraction* C = new Attraction("C");
    Attraction* D = new Attraction("D");
    Attraction* E = new Attraction("E");

    A->addNeighbor(B, 1);
    A->addNeighbor(C, 4);
    B->addNeighbor(C, 2);
    B->addNeighbor(D, 5);
    C->addNeighbor(D, 1);
    D->addNeighbor(E, 3);

    std::vector<Attraction*> attractions = {A, B, C, D, E};

    auto greedyResult = findPathGreedy(A, E);
    auto dijkstraResult = findShortestPathDijkstra(A, E);
    auto aStarResult = findPathAStar(A, E);
    auto mst = findMST(attractions);

    std::cout << "Greedy Path: ";
    printPath(greedyResult.first);
    std::cout << "Total Time: " << greedyResult.second << " minutes" << std::endl;

    std::cout << "Dijkstra Path: ";
    printPath(dijkstraResult.first);
    std::cout << "Total Time: " << dijkstraResult.second << " minutes" << std::endl;

    std::cout << "A* Path: ";
    printPath(aStarResult.first);
    std::cout << "Total Time: " << aStarResult.second << " minutes" << std::endl;

    std::cout << "MST: ";
    for (auto& edge : mst) {
        std::cout << "(" << edge.first->name << " - " << edge.second->name << ") ";
    }
    std::cout << std::endl;

    // Clean up allocated memory
    delete A;
    delete B;
    delete C;
    delete D;
    delete E;

    return 0;
}
