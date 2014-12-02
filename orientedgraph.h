#ifndef ORIENTEDGRAPH_H
#define ORIENTEDGRAPH_H

#include <list>
#include <vector>
#include <map>
#include <limits.h>
#include <algorithm>

template <class TData = int>
class OrientedGraph
{
    enum ErrorCode {
        NoError = 0,
        ErrorVertexNotFound = -1,
        ErrorEdgeNotFound = -2,
        ErrorInternal = -3
    };


    typedef unsigned int VertexID;
    typedef std::vector<VertexID> VertexesIdsVector;

    /** Класс, представляющий вершину в графе */
    class _GraphVertex {
    public:
        _GraphVertex()
        {
        }

        VertexID id() const {
            return _vtx_id;
        }

        //---------------------------------------------------------------------
        void setId(VertexID id) {
            _vtx_id = id;
        }

        //---------------------------------------------------------------------


    private:
        VertexID _vtx_id;
    };

    typedef std::list<_GraphVertex*> VertexesPtrList;
    typedef std::map<int, _GraphVertex*> MapVertexesWithIds;
    typedef std::multimap < _GraphVertex* /* vtx_from */ , _GraphVertex* /* vtx_to */ > MultiMapIncidentVertexes;
    typedef std::pair < _GraphVertex*, _GraphVertex* > PairOfVertexes;

    //=========================================================================
    typedef unsigned int EdgeWeight;

    /** Класс, представляющий ребро в графе */
    class _GraphEdge {
    public:
        _GraphEdge(_GraphVertex* vtx_start, _GraphVertex* vtx_end, int weight, TData data = TData())
            : _vtx_start(vtx_start),
              _vtx_end(vtx_end),
              _edge_weight(weight),
              _edge_data(data)
        {
        }

        //---------------------------------------------------------------------
        _GraphVertex* vertexStart() const {
            return _vtx_start;
        }

        //---------------------------------------------------------------------
        _GraphVertex* vertexEnd() const {
            return _vtx_end;
        }

        //---------------------------------------------------------------------
        EdgeWeight weight() const {
            return _edge_weight;
        }

        //---------------------------------------------------------------------
        void setWeight(EdgeWeight weight);

        //---------------------------------------------------------------------
        TData data() const {
            return _edge_data;
        }

        //---------------------------------------------------------------------
        void setData(TData data) {
            _edge_data = data;
        }

        //---------------------------------------------------------------------


    private:
        _GraphVertex* _vtx_start;
        _GraphVertex* _vtx_end;
        EdgeWeight _edge_weight;
        TData _edge_data;
    };

    typedef std::list<_GraphEdge*> EdgesPtrList;

    //=========================================================================
    typedef EdgeWeight PathWeight;

    /** Структура, описывающая пути в графе */
    typedef struct __GraphPath {
        __GraphPath()
        {
        }

        //---------------------------------------------------------------------
        bool isEmpty() const {
            return edges.empty();
        }
        //---------------------------------------------------------------------
        PathWeight weight() const {
            typename EdgesPtrList::const_iterator iter = edges.begin();
            int total_weight = 0;
            for (; edges.end() != iter; ++iter) {
                _GraphEdge* next_edge = (*iter);
                total_weight += next_edge->weight();
            }


            return total_weight;
        }

        //---------------------------------------------------------------------
        _GraphVertex* vertexStart() const {
            if (!isEmpty()) {
                return edges.front()->vertexStart();
            }


            return NULL;
        }

        //---------------------------------------------------------------------
        _GraphVertex* vertexEnd() const {
            return edges.back()->vertexEnd();
        }

        //-------------------------------------------------------------------------

        EdgesPtrList edges;
    } _GraphPath;

    typedef std::list<_GraphPath*> PathsPtrList;
    typedef std::multimap<PairOfVertexes, _GraphPath*> MultiMapGraphPaths;

    //=========================================================================

    typedef std::map<_GraphVertex*, int> MapDjkstraDistances;
    typedef std::pair<_GraphVertex*, int> PairDjkstraDistance;

    typedef std::pair<_GraphVertex*, _GraphEdge*> PairVertexEdge;
    typedef std::map < PairOfVertexes, _GraphEdge* > MapVertexesEdges;

    //=========================================================================

public:
    /** Класс предоставляет доступ внешней программе к путям в графе */
    typedef struct _PathDescriptor {
        _PathDescriptor()
            : weight( 0 )
        {
        }
        //---------------------------------------------------------------------
        bool isEmpty() const {
            return path_vertexes.empty();
        }
        //---------------------------------------------------------------------
        VertexID vertexStartId() const {
            return path_vertexes.front();
        }

        //---------------------------------------------------------------------
        VertexID vertexEndId() const {
            return path_vertexes.back();
        }

        //---------------------------------------------------------------------
        //---------------------------------------------------------------------
        //---------------------------------------------------------------------
        //---------------------------------------------------------------------

        VertexesIdsVector path_vertexes;

        /** Вес всего пути */
        PathWeight weight;
    } PathDescriptor;
    //=========================================================================

    // Открытые методы класса :
    OrientedGraph(int vtx_count)
        : _next_vacant_vtx_id(1)
    {
        // Инициализировать запрошенное число вершин графа
        for (int i = 0; i < vtx_count; ++i) {
            createVertex();
        }
    }

    //-------------------------------------------------------------------------
    /** Создает в графе новую вершину. Возвращает ее уникальный идентификатор */
    VertexID createVertex() {
        _GraphVertex* vtx = new _GraphVertex();
        VertexID id = _nextVacantVertexId();
        vtx->setId( id );
        _graph_vertexes.insert(std::make_pair(id, vtx));


        return id;
    }

    //-------------------------------------------------------------------------
    /** Удаляет вершину из графа по ее идентификатору
     *
     * @param id Уникальный идентификатор вершины */
    ErrorCode removeVertex(VertexID id) {
        _GraphVertex* vtx = _findVertex(id);
        // Если вершина найдена
        if ( vtx ) {
            // Удалить все связанные с ней ребра
            _removeAllIncidentEdges( vtx );
            typename MapVertexesWithIds::iterator iter = _graph_vertexes.find(id);
            // Удалить вершину из графа
            _graph_vertexes.erase(iter);
            // Освободить память
            delete vtx;
        } else {
            return ErrorVertexNotFound;
        }

        return NoError;
    }

    //-------------------------------------------------------------------------
    /** Возвращает TRUE, если граф содержит указанную вершину
     *
     * @param id Уникальный идентификатор вершины */
    bool hasVertex(VertexID id) const {
        return ( _findVertex(id) != NULL );
    }

    //-------------------------------------------------------------------------
    /** Возвращает количество вершин в графе */
    int vertexesCount() const;

    //-------------------------------------------------------------------------
    /** Возвращает количество ребер в графе */
    int edgesCount() const;

    //-------------------------------------------------------------------------
    /** Создает направленное ребро в графе, между двумя указанными вершинами
     *
     * @param vtx_id_start Идентификатор начальной вершины
     * @param vtx_id_end Идентификатор конечной вершины
     * @param weight Вес ребра
     * */
    ErrorCode createEdge(VertexID vtx_id_start, VertexID vtx_id_end, int weight) {
        // Найти начальную вершину по id
        _GraphVertex* vtx_start = _findVertex(vtx_id_start);
        // Найти конечную вершину по id
        _GraphVertex* vtx_end = _findVertex(vtx_id_end);
        // Если какая-то из вершин не найдена, ребро не может быть создано
        if ( vtx_start && vtx_end ) {
            // Не допускается создание параллельных ребер
            if (!_hasEdge(vtx_start, vtx_end)) {
                // Создать ребро
                _createGraphEdge(vtx_start, vtx_end, weight);
            }
        } else {
            return ErrorVertexNotFound;
        }


        return NoError;
    }

    //-------------------------------------------------------------------------
    /** Удаляет ребро из графа, между двумя указанными вершинами
     *
     * @param vtx_id_start Идентификатор начальной вершины
     * @param vtx_id_end Идентификатор конечной вершины */
    ErrorCode deleteEdge(VertexID vtx_id_start, VertexID vtx_id_end) {
        // Найти начальную вершину по id
        _GraphVertex* vtx_start = _findVertex(vtx_id_start);
        // Найти конечную вершину по id
        _GraphVertex* vtx_end = _findVertex(vtx_id_end);
        // Есои обе вершины найдены в графе
        if(vtx_start && vtx_end) {
            // Найти ребро по заданной паре вершин
            typename MapVertexesEdges::iterator iter_edges =
                    _graph_edges.find( PairOfVertexes(vtx_start, vtx_end) );
            // Если ребро найдено
            if (_graph_edges.end() != iter_edges ) {
                _GraphEdge* edge = (*iter_edges);
                // Удалить ребро из графа
                _graph_edges.erase(iter_edges);

                /* Удалить соответствующую запись из списка инцидентных вершин:
                 *
                 * 1. получить список вершин, инцидентных вершине vtx_start */
                VertexesPtrList vtxs_incident_list = _graph_incident_vertexes[vtx_start];
                /* 2. найти в этом списке вершину vtx_end, в которую
                 * входит удаляемое ребро, и удалить ее из списка */
                typename VertexesPtrList::iterator iter_incident_vtxs =
                        std::find(vtxs_incident_list.begin(),
                                  vtxs_incident_list.end(),
                                  edge->vertexEnd());

                // Освободить память
                delete edge;
            } else {
                // Иначе сообщить, что ребро не найдено
                return ErrorEdgeNotFound;
            }
        } else {
            // Иначе сообщить, что вершина не найдена
            return ErrorVertexNotFound;
        }


        return NoError;
    }

    //-------------------------------------------------------------------------
    /** Возвращает TRUE, если ребро существует между двумя указанными вершинами
     *
     * @param vtx_id_start Идентификатор начальной вершины
     * @param vtx_id_end Идентификатор конечной вершины */
    bool hasEdge(VertexID vtx_id_start, VertexID vtx_id_end) const {
        return ( _findEdge(vtx_id_start, vtx_id_end) != NULL );
    }

    //-------------------------------------------------------------------------
    /** Назначает данные ребру
     *
     * @param vtx_id_start Идентификатор начальной вершины
     * @param vtx_id_end Идентификатор конечной вершины
     * @param data Пользовательские данные */
    ErrorCode setEdgeData(VertexID vtx_id_start, VertexID vtx_id_end, TData data) {
        _GraphEdge* edge = _findEdge(vtx_id_start, vtx_id_end);
        if (edge) {
            edge->setData(data);
        } else {
            return ErrorEdgeNotFound;
        }


        return NoError;
    }

    //-------------------------------------------------------------------------
    /** Находит кратчайший путь между двумя вершинами, если он существует
     *
     * @param vtx_id_start Идентификатор начальной вершины
     * @param vtx_id_end Идентификатор конечной вершины */
    PathDescriptor shortestPath(VertexID vtx_id_start, VertexID vtx_id_end) {
        _GraphVertex* vtx_start = _findVertex(vtx_id_start);
        _GraphVertex* vtx_end = _findVertex(vtx_id_end);
        _GraphPath* path = NULL;

        // Если обе вершины принадлежат графу
        if (vtx_start && vtx_end) {
            /* Попробовать найти кратчайший путь на основе результатов алгоритма */
            path = _findShortestPath(vtx_start, vtx_end);
        }


        return path;
    }

    //-------------------------------------------------------------------------
    /** Находит эксцентриситет для заданной исходной вершины
     *
     * Эксцентриситет - путь наибольшей длины из всех кратчайших путей,
     * которые начинаются в указанной вершине.
     * @param vtx_id_start Идентификатор вершины */
    PathDescriptor findEccentricity(VertexID vtx_start_id) {
        _GraphPath* eccentricity = NULL;
        _GraphVertex* vtx_start = _findVertex(vtx_start_id);
        if (vtx_start) {
            eccentricity = _getEccentricity(vtx_start);
        }


        return _createPathDescriptor(eccentricity);
    }

    //-------------------------------------------------------------------------
    /** Находит радиус графа */
    PathDescriptor findGraphRadius() {
        _GraphPath* path_radius = _findGraphRadius();


        return _createPathDescriptor(path_radius);
    }

    //-------------------------------------------------------------------------
    //=========================================================================

private:

    //-------------------------------------------------------------------------
    /** Функция-компаратор для поиска кратчайшего пути */
    static bool comparePathWeights(_GraphPath* left, _GraphPath* right) {
        return left->weight() < right->weight();
    }

    //-------------------------------------------------------------------------


    // Внутренные классы и структуры



    /** Класс, реализующий поиск кратчайших путей в графе от заданной начальной вершины
     *
     * Использует алгоритм Дейкстры */
    class DjkstraResults {
      public:
        static const int DistanceInfinite = INT_MAX;


        DjkstraResults(_GraphVertex* vtx_start)
            : _vtx_start(vtx_start)
        {
        }

        //---------------------------------------------------------------------
        _GraphVertex* vertexStart() const {
            return _vtx_start;
        }

        //---------------------------------------------------------------------
        int pathDistance(_GraphVertex* vtx_to) const;

        //---------------------------------------------------------------------
        PairDjkstraDistance minDistance() const {
            typename MapDjkstraDistances::const_iterator iter =
                    std::min_element(_path_distances.begin(),
                                     _path_distances.end(),
                                     compareDjkstraDistances);
            if (_path_distances.end() != iter) {
                return (*iter);
            }


            return PairDjkstraDistance(NULL, 0);
        }

        //---------------------------------------------------------------------
        PairDjkstraDistance maxDistance() const {
            typename MapDjkstraDistances::const_iterator iter =
                    std::max_element(_path_distances.begin(),
                                     _path_distances.end(),
                                     compareDjkstraDistances);

            if (_path_distances.end() != iter) {
                return (*iter);
            }


            return PairDjkstraDistance(NULL, 0);
        }

        //---------------------------------------------------------------------
        int setPathDistance(_GraphVertex* vtx_end, int distance) {
            // Конечная вершина пути должна отличаться от исходной
            if (vertexStart() != vtx_end) {
                _path_distances[vtx_end] = distance;
            }


            return NoError;
        }

        //---------------------------------------------------------------------
        VertexesPtrList shortestPathVertexesSequence(_GraphVertex* vtx_end) const {
            VertexesPtrList list_path_vtxs;
            // Поместить саму конечную вершину пути в конец списка
            list_path_vtxs.push_back(vtx_end);
            if(vtx_end) {
                _GraphVertex* vtx_curr = vtx_end;
                _GraphVertex* vtx_prev = NULL;
                while (vtx_prev = _previousPathVertex(vtx_curr)) {
                    list_path_vtxs.push_front(vtx_prev);
                    vtx_curr = vtx_prev;
                }
            }


            return list_path_vtxs;
        }

        //---------------------------------------------------------------------
        int setPreLastVertexInPath(_GraphVertex* vtx_end, _GraphVertex* vtx_prev) {
            _pre_last_path_vertexes[vtx_end] = vtx_prev;


            return NoError;
        }

        //---------------------------------------------------------------------
        /** Возвращает предшествующую указанной вершину на кратчайшем пути (если он существует) */
        _GraphVertex* _previousPathVertex(_GraphVertex* vtx_end) const {
            _GraphVertex* vtx_prev = NULL;

            typename std::map <_GraphVertex*, _GraphVertex* >::const_iterator iter =
                    _pre_last_path_vertexes.find(vtx_end);
            if (_pre_last_path_vertexes.end() != iter) {
                vtx_prev = iter->second;
            }


            return vtx_prev;
        }

        //---------------------------------------------------------------------
        bool isEmpty() const {
            return (vertexStart() == NULL);
        }

        //---------------------------------------------------------------------
        /** Возвращает эксцентриситет вершины vtx_start.
         *
         * Эксцентриситет - путь с наибольшим весом из всех найденных кратчайших путей */

        //---------------------------------------------------------------------
        //---------------------------------------------------------------------
        //---------------------------------------------------------------------
        /** Функция-компаратор для поиска наименьшего значения метки в процессе работы алгоритма Дейкстры */
        static bool compareDjkstraDistances(PairDjkstraDistance left, PairDjkstraDistance right) {
            return left.second < right.second;
        }

        //---------------------------------------------------------------------

    private:
        //---------------------------------------------------------------------

        /** Указатель на граф, для которого записаны результаты работы алгоритма */
        OrientedGraph* _graph;

        /** Начальная вершина, из которой исходят все найденные пути */
        _GraphVertex* _vtx_start;

        /** Вершины-ключи данного множества являются конечными точками соответствующих
         * кратчайших путей из вершины vtx_start. Значения, хранящиеся в данном множестве,
         * представляют предпоследние точки этих кратчайших маршрутов: таким образом,
         * зная для каждой вершины предшествующую ей на пути вершину, методом последовательного
         * обхода можно восстановить любой из найденных путей полностью */
        std::map <_GraphVertex* /* vtx_to */, _GraphVertex* /* vtx_pre_last_in_path */ > _pre_last_path_vertexes;

        /** Числовые метки вершин, каждая из которых представляет
         * собой сумму весов ребер, образующих кратчайщий путь из
         * вершины vtx_start к вершине vtx_to, являющейся ключом
         * множества */
        MapDjkstraDistances _path_distances;
    };

    typedef std::map<_GraphVertex* /* vtx_from */,
    DjkstraResults*> MapVertexesDjkstraResults;

    // Закрытые методы класса

    //-------------------------------------------------------------------------
    /** Возвращает указатель на вершину с заданным идентификатором.
     *
     * Возвращает NULL-указатель, если такой вершины нет в графе.
     *
     * @param id Идентификатор вершины */
    _GraphVertex* _findVertex(int id) const {
        typename MapVertexesWithIds::const_iterator iter = _graph_vertexes.find(id);
        if (_graph_vertexes.end() != iter) {
            return iter->second;
        }

        return NULL;
    }

    //-------------------------------------------------------------------------
    /** Вычисляет кратчайшие пути в графе от заданной вершины до всех остальных
     *
     * Метод следует алгоритму Дейкстры при работе
     *
     * @param vtx_start Вершина, с которой будут начинаться все пути */
    DjkstraResults* _findAllShortestPathsFrom(_GraphVertex *vtx_start) const {
        DjkstraResults* djkstra_results = new DjkstraResults(vtx_start);

        /* Множество "еще не посещенных" вершин */
        std::map <_GraphVertex*, int> vtxs_non_visited;

        // Если указанная вершина принадлежит графу
        if ( _hasVertex( vtx_start ) ) {

            /* Инициализировать список "еще не посещенных" вершин графа и
             * присвоить каждой вершине метки с числовыми значениями:
             * 0 - для исходной вершины, бесконечность - для остальных */
            typename MapVertexesWithIds::const_iterator iter_all_vtxs = _graph_vertexes.begin();
            for (; iter_all_vtxs != _graph_vertexes.end(); ++iter_all_vtxs) {
                _GraphVertex* vtx_next = iter_all_vtxs->second;
                int distance = (vtx_start == vtx_next) ? 0 : DjkstraResults::DistanceInfinite;
                vtxs_non_visited[vtx_next] = distance;
//                djkstra_results->setPathDistance(vtx_next, distance);
            }


            /* До тех пор, пока еще остались непосещенные вершины: */
            while (!vtxs_non_visited.empty()) {
                /* Взять следующую из непосещенных вершин с наименьшим значением
                 * числовой метки */
                typename MapDjkstraDistances::iterator iter_min_dist = vtxs_non_visited.begin();
                typename MapDjkstraDistances::iterator iter_next_dist = iter_min_dist;
                ++iter_next_dist;
                for(; vtxs_non_visited.end() != iter_next_dist; ++iter_next_dist) {
                    if (iter_next_dist->second < iter_min_dist->second) {
                        iter_min_dist = iter_next_dist;
                    }
                }
                _GraphVertex* vtx_curr = iter_min_dist->first;
                int distance_to_vtx_curr = vtxs_non_visited[vtx_curr];

                /* Рассматривать соседей следующей вершины только если
                 * вершина имеет метку отличную от бесконечности */
                if (DjkstraResults::DistanceInfinite != distance_to_vtx_curr) {
                    /* Последовательно пройти всех соседей взятой вершины, вычисляя
                     * вес нового пути, проходящего через эту вершину как сумму значения
                     * числовой метки взятой вершины и веса исходящего ребра от этой
                     * вершины до соседа. */
                    EdgesPtrList outgoing_edges = _outgoingEdges(vtx_curr);
                    typename EdgesPtrList::iterator edges_iter = outgoing_edges.begin();

                    // Проходить по списку ребер, выходящих из рассматриваемой вершины
                    for (; edges_iter != outgoing_edges.end(); ++edges_iter) {
                        _GraphEdge* next_outgoing_edge = (*edges_iter);
                        // Получить вершину-соседа, в которую входит ребро
                        _GraphVertex* vtx_sibling = next_outgoing_edge->vertexEnd();

                        // Если вершина-сосед еще не помечена как посещенная
                        MapDjkstraDistances::iterator iter_vtx_non_visited =
                                vtxs_non_visited.find(vtx_sibling);
                        if (vtxs_non_visited.end() != iter_vtx_non_visited) {
                            /* Рассчитать новую длину пути к вершине-соседу через
                             * рассматриваемую вершину как сумму расстояния до
                             * рассматриваемой вершины и вес(длину) ребра между
                             * рассматриваемой вершиной и вершиной-соседом. */
                            int new_distance_calculated = distance_to_vtx_curr + next_outgoing_edge->weight();

                            /* Сравнить полученное расстояние с кратчайшим из известных на данный момент.
                             *
                             * Если результат вычислений оказывается меньше
                             * текущего значения числовой метки соседа, значит только что был
                             * найден более короткий путь к этому соседу через взятую вершину. */
                            int current_distance_to_sibling = vtxs_non_visited[vtx_sibling];
                            if (DjkstraResults::DistanceInfinite == current_distance_to_sibling ||
                                    new_distance_calculated < current_distance_to_sibling) {

                                /* Отметим взятую вершину как предпоследнюю на пути к этому соседу и
                                 * присвоим сумму-результат как новое значение числовой метки */
                                djkstra_results->setPreLastVertexInPath(vtx_sibling, vtx_curr);
                                vtxs_non_visited[vtx_sibling] = new_distance_calculated;
                            }
                        }
                    }
                }

                /* После того как все соседи рассматриваемой вершины обработаны,
                 * отмечаем эту вершину как посещенную */
                vtxs_non_visited.erase(vtx_curr);

                /* Если метка вершины отлична от бесконечности, сохраним ее значение
                 * как длину кратчайшего пути до нее */
                if (DjkstraResults::DistanceInfinite != distance_to_vtx_curr) {
                    djkstra_results->setPathDistance(vtx_curr, distance_to_vtx_curr);
                }
            }
        }


        return djkstra_results;
    }

    //-------------------------------------------------------------------------
    bool _hasVertex(_GraphVertex* vtx) const {
        return (vtx && hasVertex(vtx->id()));
    }

    //-------------------------------------------------------------------------
    ErrorCode _addIncidentVertex(_GraphVertex* vtx_start, _GraphVertex* vtx_end) {
        // Если обе вершины принадлежат текущему графу
        if( _hasVertex(vtx_start) && _hasVertex(vtx_end) ) {
            // Добавить вершины в список инцидентности
            _graph_incident_vertexes.insert( std::make_pair(vtx_start, vtx_end) );
        } else {
            return ErrorVertexNotFound;
        }


        return NoError;
    }

    //-------------------------------------------------------------------------
    int _nextVacantVertexId() {
        int id = _next_vacant_vtx_id++;

        return id;
    }

    //-------------------------------------------------------------------------
    bool _isVertexBelongsToAnyEdge(_GraphVertex* vtx);

    //-------------------------------------------------------------------------
    void _removeAllIncidentEdges(_GraphVertex* vtx);

    //-------------------------------------------------------------------------
    EdgesPtrList _outgoingEdges(_GraphVertex* vtx_start) const {
        EdgesPtrList out_edges_list;

        if (vtx_start) {
            typename MultiMapIncidentVertexes::const_iterator iter = _graph_incident_vertexes.begin();
            for (; _graph_incident_vertexes.end() != iter; ++iter)
            {
                if (iter->first == vtx_start) {
                    _GraphVertex* vtx_end = iter->second;
                    _GraphEdge* edge = _findEdge(vtx_start->id(), vtx_end->id());
                    if (edge) {
                        out_edges_list.push_back(edge);
                    }
                }
            }
        }


        return out_edges_list;
    }

    //-------------------------------------------------------------------------
    _GraphEdge* _findEdge(VertexID vtx_id_start, VertexID vtx_id_end) const {
        _GraphVertex* vtx_start = _findVertex(vtx_id_start);
        _GraphVertex* vtx_end = _findVertex(vtx_id_end);
        // Если обе вершины принадлежат графу
        if (vtx_start && vtx_end) {
            /* Найти ребро в списке:
             * если оно существует, итератор будет указывать на соответствующую запись,
             * иначе - на конец контейнера */
            typename MapVertexesEdges::const_iterator iter_edges = _graph_edges.find( PairOfVertexes(vtx_start, vtx_end));
            if ( _graph_edges.end() != iter_edges ) {
                _GraphEdge* edge = iter_edges->second;
                return edge;
            }
        }

        return NULL;
    }

    //-------------------------------------------------------------------------
    bool _hasEdge(_GraphEdge* edge) const {
        if(edge) {
            PairOfVertexes pair_vtxs(edge->vertexStart(), edge->vertexEnd());
            typename MapVertexesEdges::iterator iter =
                    std::find(_graph_edges.begin(),
                              _graph_edges.end(),
                              pair_vtxs);

            return _graph_edges.end() != iter;
        }


        return false;
    }

    //---------------------------------------------------------------------
//    bool _hasPath(_GraphPath* path) const {
//        if(path) {
//            typename PathsPtrList::iterator iter =
//                    std::find(_graph_paths.begin(),
//                              _graph_paths.end(),
//                              path);

//            return _graph_paths.end() != iter;
//        }


//        return false;
//    }

    //-------------------------------------------------------------------------
    DjkstraResults* _cachedDjkstraResults(_GraphVertex* vtx_start) const {
        DjkstraResults* results = NULL;

        typename MapVertexesDjkstraResults::const_iterator iter_cached_djkstra =
                __cache__djkstra_results.find(vtx_start);
        // Если в кэше присутствуют результаты работы алгоритма Дейкстры
        if (__cache__djkstra_results.end() != iter_cached_djkstra) {

            // Воспользоваться кэшированными резульатами
            results = iter_cached_djkstra->second;
        } else {
            // Получить новые результаты и сохранить их в кэше
            results = _findAllShortestPathsFrom(vtx_start);
            __cache__djkstra_results.insert( std::make_pair(vtx_start, results) );
        }


        return results;
    }

    //-------------------------------------------------------------------------
    _GraphPath* _findShortestPath(_GraphVertex* vtx_end, DjkstraResults* djkstra_results) const {
        _GraphPath* path = NULL;
        VertexesPtrList list_path_vtxs = djkstra_results->shortestPathVertexesSequence(vtx_end);
        // Если список вершин пути успешно найден и содержит хотя бы одну пару вершин
        if (1 < list_path_vtxs.size()) {
            path = new _GraphPath();
            _GraphVertex* next_vtx_from = list_path_vtxs.front();
            list_path_vtxs.pop_front();
            _GraphVertex* next_vtx_to = NULL;
            /* Последовательно проходя пары следующих друг за другом вершин
             * добавлять объекты-ребра к пути */
            while (!list_path_vtxs.empty()) {
                next_vtx_to = list_path_vtxs.front();
                list_path_vtxs.pop_front();
                _GraphEdge* edge = _findEdge(next_vtx_from->id(), next_vtx_to->id());
                path->edges.push_back(edge);

                next_vtx_from = next_vtx_to;
            }
        }

        return path;
    }

    //---------------------------------------------------------------------
    _GraphPath* _findShortestPath(_GraphVertex* vtx_start, _GraphVertex* vtx_end) const {
        DjkstraResults* djkstra_results = _cachedDjkstraResults(vtx_start);

        return _findShortestPath(vtx_end, djkstra_results);
    }

    //---------------------------------------------------------------------
//    /** Продолжает путь, добавляя в его конец указанное ребро */
//    ErrorCode _appendEdgeToPath(_GraphEdge* edge, _GraphPath* path) {
//        // Если указатнль на объект - не NULL,
//        if (path &&
//                // путь принадлежит графу,
//                _hasPath(path) &&
//                // и ребро принадлежит графу
//                _hasEdge(edge) ) {

//            // Если путь еще не содержит ребер
//            if (path->isEmpty() ||
//                    /* или ребро выходит из вершины, которая является концом пути */
//                    path->vertexEnd() == edge->vertexStart() ) {
//                // Продолжить путь по ребру
//                path->edges.push_back(edge);
//            }
//        }


//        return NoError;
//    }

    //-------------------------------------------------------------------------
//    /** Присоединяет ребро к началу пути
//     *
//     * После присоединения путь будет иметь начало в вершине, из которой
//     * выходит ребро и, пройдя по нему, продолжится по ранее существовавшему
//     * маршруту */
//    ErrorCode _prependEdgeToPath(_GraphEdge* edge, _GraphPath* path) {
//        // Если указатнль на объект - не NULL,
//        if (path &&
//                // путь принадлежит графу,
//                _hasPath(path) &&
//                // и ребро принадлежит графу
//                _hasEdge(edge) ) {

//            // Если путь еще не содержит ребер
//            if (path->isEmpty() ||
//                    /* или путь начинается из вершины, в которую входит ребро */
//                    path->vertexStart() == edge->vertexEnd()) {
//                // Присоединить ребро к началу пути
//                path->edges.push_front(edge);
//            }
//        }


//        return NoError;
//    }

    //-------------------------------------------------------------------------
    _GraphPath* _findGraphRadius() const {
        /* Список эксцентриситетов для всех последовательно рассматриваемых вершин */
        std::list <_GraphPath*> eccentricities;

        /* Для каждой вершины графа найти все кратчайшие пути от нее
         * до остальных вершин графа по алгоритму Дейкстры */
        DjkstraResults* djkstra_results = NULL;
        typename MapVertexesWithIds::const_iterator iter_all_vtxs = _graph_vertexes.begin();
        for (; iter_all_vtxs != _graph_vertexes.end(); ++iter_all_vtxs) {
            _GraphVertex* next_vtx_start = iter_all_vtxs->second;
            djkstra_results = _cachedDjkstraResults(next_vtx_start);
            /* Из каждого списка кратчайших путей выделить эксцентриситет - самый
             * длинный путь */
            _GraphPath* next_eccentricity = _getEccentricity( djkstra_results->vertexStart() );
            /* Рассматриваемая вершина может и не иметь исходящих ребер -
             * в этом случае из нее не может быть построено ни одного
             * пути, поэтому запрос эксцентриситета вернет NULL-указатель */
            if (next_eccentricity) {
                eccentricities.push_back(next_eccentricity);
            }
        }

        /* Из всех эксцентриситетов выбрать кратчайший - это и есть
         * искомый радиус графа */
        typename PathsPtrList::iterator iter_graph_radius =
                std::min_element(eccentricities.begin(), eccentricities.end(), comparePathWeights);
        _GraphPath* graph_radius = NULL;
        if (eccentricities.end() != iter_graph_radius) {
            graph_radius = (*iter_graph_radius);
        }

        /* Освободить память от ненужных объектов, предварительно
         * удалив из списка указатель на объект-радиус во избежание его уничтожения */
        eccentricities.erase(iter_graph_radius);
        while (!eccentricities.empty()) {
            _GraphPath* path = eccentricities.front();
            eccentricities.pop_front();

            delete path;
        }


        return graph_radius;
    }

    //-------------------------------------------------------------------------
    PathDescriptor _createPathDescriptor(_GraphPath* path) {
        PathDescriptor descriptor;
        // Если объект-путь не пустой
        if (path && !path->isEmpty()) {
            // Составить список из идентификаторов его последовательных вершин
            descriptor.path_vertexes.push_back(path->vertexStart()->id());
            EdgesPtrList::iterator iter_path_edges = path->edges.begin();
            for (; path->edges.end() != iter_path_edges; ++iter_path_edges) {
                _GraphEdge* next_edge = (*iter_path_edges);
                descriptor.path_vertexes.push_back( next_edge->vertexEnd()->id() );
            }

            // Установить вес пути
            descriptor.weight = path->weight();
        }


        return descriptor;
    }

    //-------------------------------------------------------------------------
    /** Возвращает эксцентриситет вершины
     *
     * Эксцентриситет - максимальное расстояние из всех кратчайших расстояний
     * от данной вершины до других вершин графа.
     * @param vtx_start Исходная вершина */
    _GraphPath* _getEccentricity(_GraphVertex* vtx_start) const {
        _GraphPath* path = NULL;
        if(vtx_start) {
            // Получить список кратчайших путей (результаты работы алгоритма Дейкстры)
            DjkstraResults* djkstra_results = _cachedDjkstraResults(vtx_start);
            if (djkstra_results &&
                    !djkstra_results->isEmpty()) {
                // Найти максимальное расстояние
                PairDjkstraDistance pair_max_distance = djkstra_results->maxDistance();
                // Найти кратчайший путь, соответствующий этому расстоянию
                _GraphVertex* vtx_path_end = pair_max_distance.first;
                path = _findShortestPath(vtx_path_end, djkstra_results);
            }
        }

        return path;
    }

    //-------------------------------------------------------------------------
    void _createGraphEdge(_GraphVertex* vtx_start, _GraphVertex* vtx_end, int weight)
    {
        _GraphEdge* edge = new _GraphEdge(vtx_start, vtx_end, weight);
        // Добавить ребро в список
        PairOfVertexes pair_vtxs(vtx_start, vtx_end);
        _graph_edges.insert( std::make_pair( pair_vtxs, edge ) );

        // Добавить новую вершину в список инцидентности графа
        _addIncidentVertex(vtx_start, vtx_end);
    }

    //-------------------------------------------------------------------------
    bool _hasEdge(_GraphVertex* vtx_start, _GraphVertex* vtx_end) {
        return ( _graph_edges.find( PairOfVertexes(vtx_start, vtx_end) )
                 != _graph_edges.end() );
    }

    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------


    // Переменные класса

    /** Множество вершин графа */
    MapVertexesWithIds _graph_vertexes;

    /** Список смежности вершин:
     *
     * Ключами множества являются вершины, из которых выходят ребра,
     * а элементами списка являются вешины, в которые эти ребра входят */
    MultiMapIncidentVertexes _graph_incident_vertexes;

    /** Множество объектов-ребер графа.
     *
     * Ключами множества являются пары вершин начальная-конечная, между которыми существуют ребра.
     * Тип ключа - пара вершин начальная/конечная ( std::pair <Vertex*, Vertex*> )
     * Тип значения - Ребро ( Edge* ) */
    MapVertexesEdges _graph_edges;

    /** Переменная для хранения кэшированных результатов работы
     * алгоритма Дейкстры */
    mutable MapVertexesDjkstraResults __cache__djkstra_results;

    VertexID _next_vacant_vtx_id;


};

#endif // ORIENTEDGRAPH_H
