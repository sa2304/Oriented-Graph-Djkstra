#ifndef TESTORIENTEDGRAPH_H
#define TESTORIENTEDGRAPH_H

#include <iostream>
#include "orientedgraph.h"

class TestOrientedGraph
{
public:
    TestOrientedGraph();

    void run();


    static bool testGraph1();
    static bool testGraph2();

private:
    static OrientedGraph<>* _s_createTestGraph1();
    static OrientedGraph<>* _s_createTestGraph2();
};

#endif // TESTORIENTEDGRAPH_H
