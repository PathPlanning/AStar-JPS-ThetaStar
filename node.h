#ifndef NODE_H
#define NODE_H

#include <functional>

struct Node
{
    int     i, j;
    double  F, g, H;
    Node    *parent;

    bool operator==(const Node &other) const {
        return i == other.i && j == other.j;
    }
};

namespace std {
    template<>
    struct hash<Node> {
        size_t operator()(const Node &x) const {
            size_t seed = 0;
            seed ^= std::hash<int>()(x.i) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(x.j) + 0x9e3779b9
                    + (seed << 6) + (seed >> 2);

            return seed;
        }
    };
}
#endif
