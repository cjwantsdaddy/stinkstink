#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
#include <climits>
#include <unordered_map>
#include <set>

using namespace std;

class Attraction {
public:
    string name;
    map<Attraction*, int> neighbors;
    Attraction(string name) : name(name) {}

    void addNeighbor(Attraction* neighbor, int time) {
        neighbors[neighbor] = time;
    }
};
pair<vector<Attraction*>, int> findPathGreedy(Attraction* start, Attraction* end) {
    vector<Attraction*> path;
    set<Attraction*> visited;
    Attraction* current = start;
    int totalTime = 0;

    while (current != end) {
        path.push_back(current);
        visited.insert(current);
       
        int shortestTime = numeric_limits<int>::max();
        Attraction* nextAttraction = nullptr;

        for (auto& neighbor : current->neighbors) {
            if (visited.find(neighbor.first) == visited.end() && neighbor.second < shortestTime) {
                shortestTime = neighbor.second;
                nextAttraction = neighbor.first;
            }
        }

        if (nextAttraction == nullptr) {
            return {{}, -1}; 
        }

        current = nextAttraction;
        totalTime += shortestTime;
    }

    path.push_back(end);
    return {path, totalTime};
}
   pair<vector<Attraction*>, int> findShortestPathDijkstra(Attraction* start, Attraction* end) {
    map<Attraction*, int> distances;
    map<Attraction*, Attraction*> previous;
    set<Attraction*> visited;
    priority_queue<pair<int, Attraction*>, vector<pair<int, Attraction*>>, greater<pair<int, Attraction*>>> pq;

    for (auto& neighbor : start->neighbors) {
        distances[neighbor.first] = numeric_limits<int>::max();
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

    vector<Attraction*> path;
    int totalTime = distances[end];
    for (Attraction* at = end; at != nullptr; at = previous[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    return {path, totalTime};
}

int heuristic(Attraction* a, Attraction* b) {
    return 1; 
}

pair<vector<Attraction*>, int> findPathAStar(Attraction* start, Attraction* end) {
    map<Attraction*, int> gScore;
    map<Attraction*, int> fScore;
    map<Attraction*, Attraction*> previous;
    set<Attraction*> openSet;
    priority_queue<pair<int, Attraction*>, vector<pair<int, Attraction*>>, greater<pair<int, Attraction*>>> pq;

    gScore[start] = 0;
    fScore[start] = heuristic(start, end);
    pq.push({fScore[start], start});
    openSet.insert(start);

    while (!pq.empty()) {
        Attraction* current = pq.top().second;
        pq.pop();

        if (current == end) {
            vector<Attraction*> path;
            int totalTime = gScore[end];
            for (Attraction* at = end; at != nullptr; at = previous[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
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

    return {{}, -1};
}
vector<pair<Attraction*, Attraction*>> findMST(vector<Attraction*> attractions) {
    vector<pair<Attraction*, Attraction*>> mst;
    set<Attraction*> inMST;
    priority_queue<pair<int, pair<Attraction*, Attraction*>>, vector<pair<int, pair<Attraction*, Attraction*>>>, greater<pair<int, pair<Attraction*, Attraction*>>>> pq;

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

    vector<Attraction*> attractions = {A, B, C, D, E};

    auto greedyResult = findPathGreedy(A, E);
    auto dijkstraResult = findShortestPathDijkstra(A, E);
    auto aStarResult = findPathAStar(A, E);
    auto mst = findMST(attractions);

    cout << "Greedy Path: ";
    printPath(greedyResult.first);
    cout << "Total Time: " << greedyResult.second << " minutes" << endl;

    cout << "Dijkstra Path: ";
    printPath(dijkstraResult.first);
    cout << "Total Time: " << dijkstraResult.second << " minutes" << endl;

    cout << "A* Path: ";
}
