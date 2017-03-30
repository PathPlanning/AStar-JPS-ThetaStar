#ifndef NODE_H
#define NODE_H

struct Node
{
    int     i, j;
    double  F, g, H;
    Node    *parent;

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
};
#endif
