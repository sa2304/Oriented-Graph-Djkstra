#include "testorientedgraph.h"

TestOrientedGraph::TestOrientedGraph()
{
}

//-----------------------------------------------------------------------------
void TestOrientedGraph::run() {
    std::cout << "Running testGraph1... ";
    if ( testGraph1() ) {
        std::cout << "Ok!";
    } else {
        std::cout << "FAILED";
    }
    std::cout << std::endl;


    std::cout << "Running testGraph2... ";
    if ( testGraph2() ) {
        std::cout << "Ok!";
    } else {
        std::cout << "FAILED";
    }
    std::cout << std::endl;


    getchar();
}

//-----------------------------------------------------------------------------
bool TestOrientedGraph::testGraph1() {
//    OrientedGraph<>* graph = _s_createTestGraph1();

//    OrientedGraph<>::PathDescriptor radius = graph->findGraphRadius();

//    /* В соответствии с произведенными вычислениями полученным радиусом
//     * должен являться путь 3->4 с суммарным весом 16. */
//    bool is_radius_start_vertex_3 = ( 3 == radius.vertexStartId() );
//    bool is_radius_end_vertex_4 = ( 4 == radius.vertexEndId() );
//    bool is_radius_total_weight_16 = ( 16 == radius.weight );

//    return (is_radius_start_vertex_3 &&
//            is_radius_end_vertex_4 &&
//            is_radius_total_weight_16);


    return false;
}

//-----------------------------------------------------------------------------
bool TestOrientedGraph::testGraph2() {
//    OrientedGraph<>* graph2 = _s_createTestGraph2();
//    OrientedGraph<>::PathDescriptor radius = graph2->findGraphRadius();

//    /* В соответствии с произведенными вычислениями полученным радиусом
//     * должен являться путь 1->2->3->6->5->4 с суммарным весом 53 */
//    bool is_all_radius_vertexes_correct = (
//                radius.path_vertexes[0] == 1 &&
//                radius.path_vertexes[1] == 2 &&
//                radius.path_vertexes[2] == 5 &&
//                radius.path_vertexes[3] == 4
//            );

//    bool is_radius_weight_correct = (53 == radius.weight);


//    return is_all_radius_vertexes_correct && is_radius_weight_correct;


    return false;
}


//-----------------------------------------------------------------------------
OrientedGraph<int> *TestOrientedGraph::_s_createTestGraph1() {
    OrientedGraph<>* graph = new OrientedGraph<>(5);
    graph->createEdge(1, 2, 10);
    graph->createEdge(1, 3, 40);
    graph->createEdge(1, 5, 24);
    graph->createEdge(2, 3, 20);
    graph->createEdge(2, 4, 7);
    graph->createEdge(3, 4, 16);
    graph->createEdge(5, 4, 33);


    return graph;
}

//-----------------------------------------------------------------------------
OrientedGraph<>* TestOrientedGraph::_s_createTestGraph2() {
    OrientedGraph<>* graph = new OrientedGraph<>(6);
    graph->createEdge(1, 2, 2);
    graph->createEdge(1, 6, 30);
    graph->createEdge(2, 3, 12);
    graph->createEdge(2, 5, 33);
    graph->createEdge(3, 6, 50);
    graph->createEdge(6, 5, 22);
    graph->createEdge(5, 4, 18);
    graph->createEdge(4, 1, 26);
    graph->createEdge(4, 2, 14);


    return graph;

}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
