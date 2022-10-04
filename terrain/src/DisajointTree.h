#ifndef DISAJOINT_TREE_H_
#define DISAJOINT_TREE_H_

#include <vector>

// disajoint tree for merging sets
class DisajointTree {
   public:
    DisajointTree() {}
    DisajointTree(int n) {
        parent.resize(n);
        rank.resize(n, 1);
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    int Parent(int x) {
        if (x == parent[x]) return x;
        int y = Parent(parent[x]);
        parent[x] = y;
        return y;
    }
    int Index(int x) { return indices[x]; }
    int IndexToParent(int x) {return indices_to_parent[x]; };
    void MergeFromTo(int x, int y) {
        int px = Parent(x);
        int py = Parent(y);
        if (px == py) return;
        rank[py] += rank[px];
        parent[px] = py;
    }
    void Merge(int x, int y) {
        int px = Parent(x);
        int py = Parent(y);
        if (px == py) return;
        if (rank[px] < rank[py]) {
            rank[py] += rank[px];
            parent[px] = py;
        } else {
            rank[px] += rank[py];
            parent[py] = px;
        }
    }

    // renumber the root so that it is consecutive.
    void BuildCompactParent() {
        std::vector<int> compact_parent;
        compact_parent.resize(parent.size());
        compact_num = 0;
        for (int i = 0; i < parent.size(); ++i) {
            if (parent[i] == i) {
                compact_parent[i] = compact_num++;
                indices_to_parent.push_back(i);
            }
        }
        indices.resize(parent.size());
        for (int i = 0; i < parent.size(); ++i) {
            indices[i] = compact_parent[Parent(i)];
        }
    }

    int CompactNum() { return compact_num; }

    int compact_num;
    std::vector<int> parent;
    std::vector<int> indices, indices_to_parent;
    std::vector<double> rank;
};


#endif