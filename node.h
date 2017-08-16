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
    Node(int i=-1, int j=-1, double F=-1, double g=-1, double H=-1, Node* parent = nullptr):i(i),j(j),F(F),g(g),H(H),parent(parent){}
};
#endif
