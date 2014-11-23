#ifndef ORIENTEDGRAPH_H
#define ORIENTEDGRAPH_H

#include <list>
#include <set>
#include <limits.h>

template <class TWeight, class TData>
class OrientedGraph
{
public:
    class Vertex {
    public:
        Vertex();

        int id() const;
        void setId(int id);

    private:
        int _vtx_id;

    };


    class Egde {
    public:
        Edge(Vertex* vtx_start, Vertex* vtx_end, TWeight weight, TData data);

        Vertex* vertexStart() const;
        Vertex* vertexEnd() const;
        TWeight weight() const;
        void setWeight(TWeight weight);

    private:
        Vertex* _vtx_start;
        Vertex* _vtx_end;
        TWeight _edge_weight;
    };

    /// TODO :
    class Path {
    public:
        Path();

    };


    OrientedGraph(int vtx_count);

//    void insertVertex(Vertex* vtx);
//    void removeVertex(Vertex* vtx);
//    bool hasVertex(Vertex* vtx);


    int vertexesCount() const;
    int edgesCount() const;

    int insertEdge(Vertex* vtx_start, Vertex* vtx_end, TWeight weight);
    int deleteEdge(Vertex* vtx_start, Vertex* vtx_end);
    bool isEdgeExists(Vertex* vtx_start, Vertex* vtx_end) const;

    int setEdgeData(Vertex* vtx_start, Vertex* vtx_end, TData data);



private:

    // Внутренные классы и структуры


    /** Класс, реализующий поиск кратчайших путей в графе от заданной начальной вершины
     *
     * Использует алгоритм Дейкстры */
    class DjkstraResults {
      public:
        DjkstraResults(Vertex* vtx_start);

    private:
        /** Начальная вершина, из которой исходят все найденные пути */
        Vertex* _vtx_srart;

        /** Вершины-ключи данного множества являются конечными точками соответствующих
         * кратчайших путей из вершины vtx_start. Значения, хранящиеся в данном множестве,
         * представляют предпоследние точки этих кратчайших маршрутов: таким образом,
         * зная для каждой вершины предшествующую ей на пути вершину, методом последовательного
         * обхода можно восстановить любой из найденных путей полностью */
        std::set <Vertex* /* vtx_to */, Vertex* /* vtx_pre_last_in_path */ > _pre_last_path_vertexes;

        /** Числовые метки вершин, каждая из которых представляет
         * собой сумму весов ребер, образующих кратчайщий путь из
         * вершины vtx_start к вершине vtx_to, являющейся ключом
         * множества */
        std::set <Vertex* /* vtx_to */, int /* path_total_cost */> _path_costs;
    };


    // Закрытые методы класса

    DjkstraResults _findAllShortestPathsFrom(Vertex *vtx_start) const;

    Path* _findGraphRadius() const;


    // Переменные класса

    /** Список смежности вершин:
     *
     * Ключами множества являются вершины, из которых выходят ребра,
     * а элементами списка являются вешины, в которые эти ребра входят */
    std::set < Vertex*, list<Vertex*> > _graph_incident_vertexes;

    /** Множество объектов-ребер графа.
     *
     * Ключами множества являются вершины, из которых выходят эти ребра */
    std::set < Vertex*, list<Egde*> > _graph_outgoing_edges;


    /** Переменная для хранения кэшированных результатов работы метода _findAllShortestPathsFrom()
     *
     * @see _findAllShortestPathsFrom */
    std::set < Vertex*, std::list<Path*> > __cached__shortest_paths;
};

#endif // ORIENTEDGRAPH_H
