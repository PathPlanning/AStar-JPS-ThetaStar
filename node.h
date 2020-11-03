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

struct FocalElem
{
    int i,j;
    double h, g, F;
    FocalElem(const Node& node) {i = node.i; j = node.j; F = node.F; g = node.g; }
};

#endif
