#ifndef ORIENTEDGRAPH_H
#define ORIENTEDGRAPH_H

#include <list>
#include <map>
#include <limits.h>
#include <algorithm>

template <class  = int, class TData = int>
class OrientedGraph
{
public:
    class Vertex {
    public:
        Vertex()
        {
        }

        int id() const;
        void setId(int id);

    private:
        int _vtx_id;
    };


    class Edge {
    public:
        Edge(Vertex* vtx_start, Vertex* vtx_end,  weight, TData data);

        Vertex* vertexStart() const;
        Vertex* vertexEnd() const;
         weight() const;
        void setWeight(int weight);

    private:
        Vertex* _vtx_start;
        Vertex* _vtx_end;
        int _edge_weight;
    };

    /// TODO :
    class Path {
    public:
        Path();

    };


    OrientedGraph(int vtx_count) {
        // Инициализировать запрошенное число вершин графа
        for (int i = 0; i < vtx_count; ++i) {
            Vertex* vertex = new Vertex();
            insertVertex(vertex);
        }
    }

    void insertVertex(Vertex* vtx) {
        if (vtx && !vtx->id()) {
            vtx->setId( _nextVacantVertexId() );
            _graph_vertexes.push_back(vtx);
        }
    }
    void removeVertex(Vertex* vtx) {
        if ( hasVertex(vtx) ) {
            _removeAllIncidentEdges(vtx);
            _graph_vertexes.remove(vtx);
        }
    }
    bool hasVertex(Vertex* vtx) const {
        return (
                    std::find(_graph_vertexes.begin(),
                              _graph_vertexes.end(),
                              vtx) != _graph_vertexes.end()
                );
    }


    int vertexesCount() const;
    int edgesCount() const;

    int insertEdge(Vertex* vtx_start, Vertex* vtx_end, int weight);
    int deleteEdge(Vertex* vtx_start, Vertex* vtx_end);
    bool isEdgeExists(Vertex* vtx_start, Vertex* vtx_end) const;

    int setEdgeData(Vertex* vtx_start, Vertex* vtx_end, TData data);


    /** Функция-компаратор для поиска наименьшего значения метки в процессе работы алгоритма Дейкстры */
    static bool compareDjkstraDistances(std::pair<Vertex*, int> left, std::pair<Vertex*, int> right) {
        return left.second < right.second;
    }


private:

    // Внутренные классы и структуры


    /** Класс, реализующий поиск кратчайших путей в графе от заданной начальной вершины
     *
     * Использует алгоритм Дейкстры */
    struct DjkstraResults {
      public:
        DjkstraResults()
            : vtx_srart(NULL)
        {
        }

        /** Начальная вершина, из которой исходят все найденные пути */
        Vertex* vtx_srart;

        /** Вершины-ключи данного множества являются конечными точками соответствующих
         * кратчайших путей из вершины vtx_start. Значения, хранящиеся в данном множестве,
         * представляют предпоследние точки этих кратчайших маршрутов: таким образом,
         * зная для каждой вершины предшествующую ей на пути вершину, методом последовательного
         * обхода можно восстановить любой из найденных путей полностью */
        std::map <Vertex* /* vtx_to */, Vertex* /* vtx_pre_last_in_path */ > pre_last_path_vertexes;

        /** Числовые метки вершин, каждая из которых представляет
         * собой сумму весов ребер, образующих кратчайщий путь из
         * вершины vtx_start к вершине vtx_to, являющейся ключом
         * множества */
        std::map <Vertex* /* vtx_to */, int /* path_distance */> path_distances;
    };


    // Закрытые методы класса

    /** Вычисляет кратчайшие пути в графе от заданной вершины до всех остальных
     *
     * Метод следует алгоритму Дейкстры при работе
     *
     * @param vtx_start Вершина, с которой будут начинаться все пути */
    DjkstraResults _findAllShortestPathsFrom(Vertex *vtx_start) const {
        /* Множество пар вершин последняя-предпоследняя, по которому можно
         * полностью восстановить найденные кратчайшие пути */
        std::map<Vertex* /* последняя вершина кратчайшего пути */,
                 Vertex* /* предпоследняя вершина пути */ > vtxs_pre_last;
        std::map <Vertex*, int> vtxs_non_visited;

        /* В этот список попадают вершины после того, как окончательно найдено
         * кратчайее расстояние до них */
        std::map <Vertex*, int> vtxs_final_distances;

        // Если указанная вершина принадлежит графу
        if ( hasVertex(vtx) ) {

            /* Инициализировать список "еще не посещенных" вершин графа и
             * присвоить каждой вершине метки с числовыми значениями:
             * 0 - для исходной вершины, бесконечность - для остальных */
            std::list<Vertex*>::iterator iter = _graph_vertexes.begin();
            for (; iter != _graph_vertexes.end(); ++iter) {
                Vertex* vtx_next = (*iter);
                int distance = (vtx_start == vtx_next) ? 0 : INT_MAX;
                vtxs_non_visited[vtx_next] = distance;
            }


            /* До тех пор, пока еще остались непосещенные вершины: */
            while (!vtxs_non_visited.empty()) {
                /* Взять следующую из непосещенных вершин с наименьшим значением
                 * числовой метки */
                std::map<Vertex*, int>::iterator iter_min_dist =
                        std::min_element(vtxs_non_visited.begin(),
                                         vtxs_non_visited.end(),
                                         OrientedGraph::compareDjkstraDistances);

                Vertex* vtx_curr = (*iter_min_dist).first;

                /* Последовательно пройти всех соседей взятой вершины, вычисляя
                 * вес нового пути, проходящего через эту вершину как сумму значения
                 * числовой метки взятой вершины и веса исходящего ребра от этой
                 * вершины до соседа. */
                std::list<Edge*> outgoing_edges = _outgoingEdges(vtx_curr);
                std::list<Edge*>::iterator edges_iter = outgoing_edges.begin();
                int distance_vtx_curr = vtxs_non_visited[vtx_curr];
                for (; edges_iter != outgoing_edges.end(); ++edges_iter) {
                    Edge* next_edge = (*edges_iter);
                    Vertex* vtx_sibling = next_edge->vertexEnd();
                    int new_weight_calculated = distance_vtx_curr + next_edge->weight();
                    int distance_sibling_curr = vtxs_non_visited[vtx_sibling];
                    /* Если результат вычислений оказывается меньше
                     * текущего значения числовой метки соседа, значит только что был
                     * найден более короткий путь к этому соседу через взятую вершину. */
                    if (new_weight_calculated < distance_sibling_curr) {
                        /* Отметим взятую вершину как предпоследнюю на пути к этому соседу и
                         * присвоим сумму-результат как новое значение числовой метки */
                        vtxs_pre_last[vtx_sibling] = vtx_curr;
                        vtxs_non_visited[vtx_sibling] = new_weight_calculated;
                    }
                }

                /* После того как все соседи рассматриваемой вершины обработаны,
                 * отмечаем эту вершину как посещенную */
                vtxs_final_distances[vtx_curr] = distance_vtx_curr;
                vtxs_non_visited.erase(vtx_curr);
            }
        }

        // Собрать результаты работы алгоритма в структуру и вернуть ее
        DjkstraResults results;
        results.vtx_srart = vtx_start;
        results.pre_last_path_vertexes = vtxs_pre_last;
        results.path_distances = vtxs_final_distances;


        return results;
    }

    //-------------------------------------------------------------------------
    Path* _findGraphRadius() const {
        /* Для каждой вершины графа найти все кратчайшие пути от нее
         * до остальных вершин графа по алгоритму Дейкстры */


        /* Из каждого списка путей выделить эксцентриситет - самый
         * длинный путь */


        /* Из всех эксцентриситетов выбрать кратчайший - это и есть
         * искомый радиус графа */
    }

    //-------------------------------------------------------------------------
    int _nextVacantVertexId();

    //-------------------------------------------------------------------------
    bool _isVertexBelongsToAnyEdge(Vertex* vtx);

    //-------------------------------------------------------------------------
    void _removeAllIncidentEdges(Vertex* vtx);

    //-------------------------------------------------------------------------

    std::list<Edge*> _outgoingEdges(Vertex* vtx_from);
    //-------------------------------------------------------------------------


    // Переменные класса

    /** Множество вершин графа */
    std::list <Vertex*> _graph_vertexes;

    /** Список смежности вершин:
     *
     * Ключами множества являются вершины, из которых выходят ребра,
     * а элементами списка являются вешины, в которые эти ребра входят */
    std::map < Vertex*, list<Vertex*> > _graph_incident_vertexes;

    /** Множество объектов-ребер графа.
     *
     * Ключами множества являются вершины, из которых выходят эти ребра */
    std::map < Vertex*, list<Egde*> > _graph_outgoing_edges;

    /** Переменная для хранения кэшированных результатов работы метода _findAllShortestPathsFrom()
     *
     * @see _findAllShortestPathsFrom */
    std::map < Vertex*, std::list<Path*> > __cached__shortest_paths;

    int _next_vacant_vtx_id;


};

#endif // ORIENTEDGRAPH_H
