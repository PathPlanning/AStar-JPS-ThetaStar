#ifndef STRUCTS_H
#define STRUCTS_H
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
using namespace boost::multi_index;

struct Node
{
    int     i, j, id;
    double  f, g, h;
    Node    *parent;

    Node(int _i=-1, int _j=-1, double _g = 0, double _h = 0, int _id = -1, Node* _parent = nullptr):i(_i), j(_j), g(_g), h(_h), id(_id), parent(_parent), f(_g+_h){}

    bool operator== (const Node &other) const {
        return this->id == other.id;
    }

    bool operator< (const Node &other) const{
        if(fabs(this->f - other.f) < 1e-6)
            return this->g > other.g;
        else
            return this->f < other.f;
    }
};

struct FocalElem
{
    int id;
    double f, g, focal_h;
    FocalElem(const Node& node) { id = node.id; f = node.f; g = node.g; }
    FocalElem(const Node& node, double h) { id = node.id; f = node.f; g = node.g; focal_h = h; }

    bool operator== (const FocalElem &other) const {
        return this->id == other.id;
    }

    bool operator< (const FocalElem &other) const {
        if(fabs(this->focal_h - other.focal_h) < 1e-6)
        {
            if(fabs(this->f - other.f) < 1e-6)
                return this->g > other.g;
            else
                return this->f < other.f;
        }
        else
            return this->focal_h > other.focal_h;
    }
};

typedef multi_index_container<
        Node,
        indexed_by<
                    ordered_non_unique<identity<Node>>,
                    hashed_unique<member<Node, int, &Node::id>>
        >
> OPEN_container;

typedef multi_index_container<
        FocalElem,
        indexed_by<
                    ordered_non_unique<identity<FocalElem>>,
                    ordered_unique<member<FocalElem, int, &FocalElem::id>>
        >
> FOCAL_container;

#endif // STRUCTS_H
