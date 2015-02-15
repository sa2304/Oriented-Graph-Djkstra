#ifndef ORIENTEDGRAPH_H
#define ORIENTEDGRAPH_H

#include <list>
#include <vector>
#include <map>
#include <stack>
#include <limits.h>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include <windows.h>



#define CLEAR_SCREEN() system("cls")
#define PAUSE() system("pause")

template <class TData = int>
class OrientedGraph
{
    enum ErrorCode {
        NoError = 0,
        ErrorVertexNotFound = -1,
        ErrorEdgeNotFound = -2,
        ErrorInternal = -3
    };

public:
    typedef unsigned int VertexID;
    typedef std::vector<VertexID> VertexesIdsVector;
    typedef std::list<VertexID> VertexesList;

    typedef unsigned int EdgeWeight;
    typedef EdgeWeight PathWeight;


private:
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
        void appendVertex(VertexID vid) {
            path_vertexes.push_back(vid);
        }

        //---------------------------------------------------------------------
        void prependVertex(VertexID vid) {
            path_vertexes.push_front(vid);
        }

        //---------------------------------------------------------------------
        void print() const {
            VertexesList::const_iterator iter = path_vertexes.begin();
            std::cout << "[" << (*iter) << "]";
            ++iter;
            for (; path_vertexes.end() != iter; ++iter) {
                std::cout << "->[" << (*iter) << "]";
            }
        }

        //---------------------------------------------------------------------


        VertexesList path_vertexes;

        /** Вес всего пути */
        PathWeight weight;
    } PathDescriptor;
    //=========================================================================

    //-------------------------------------------------------------------------
    // Открытые методы класса :
    OrientedGraph(int vtx_count = 0)
        : _next_vacant_vtx_id(1)
    {
        // Инициализировать запрошенное число вершин графа
        for (int i = 0; i < vtx_count; ++i) {
            createVertex();
        }
    }

    //-------------------------------------------------------------------------
    ~OrientedGraph() {
        /* Удалить объекты-вершины */
        typename MapVertexesWithIds::iterator iter_vertexes = _graph_vertexes.begin();
        for (; _graph_vertexes.end() != iter_vertexes; ++iter_vertexes) {
            _GraphVertex* next_vertex = (*iter_vertexes).second;
            delete next_vertex;
        }


        /* Удалить объекты-ребра */
        typename MapVertexesEdges::iterator iter_edges = _graph_edges.begin();
        for (; _graph_edges.end() != iter_edges; ++iter_edges) {
            _GraphEdge* next_edge = (*iter_edges).second;
            delete next_edge;
        }
    }

    //-------------------------------------------------------------------------
    /** Возвращает TRUE, если граф не содержит ни одной вершины */
    bool isEmpty() const {
        return _graph_vertexes.empty();
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
    /** Возвращает список идентификаторов всех вершин графа */
    VertexesList vertexes() const {
        VertexesList vtxs_list;
        typename MapVertexesWithIds::const_iterator iter = _graph_vertexes.begin();
        for (; _graph_vertexes.end() != iter; ++iter) {
            VertexID vid = (*iter).first;
            vtxs_list.push_back( vid );
        }


        return vtxs_list;
    }

    //-------------------------------------------------------------------------
    /** Возвращает количество вершин в графе */
    int vertexesCount() const {
        return vertexes().size();
    }

    //-------------------------------------------------------------------------
    /** Возвращает список вершин, в которые идут ребра из заданной вершины */
    VertexesList vertexOutSiblings(VertexID vid) const {
        VertexesList vtxs_list;
        _GraphVertex *vtx = _findVertex(vid);
        if (vtx) {
            /* Пройти по всем ребрам, выходящим из заданной вершины */
            EdgesPtrList edges = _outgoingEdges(vtx);
            typename EdgesPtrList::iterator iter_edges = edges.begin();
            for (; edges.end() != iter_edges; ++iter_edges) {
                _GraphEdge* next_edge = (*iter_edges);
                /* Запомнить ID вершины, в которую входит ребро */
                vtxs_list.push_back( next_edge->vertexEnd()->id() );
            }
        }


        return vtxs_list;
    }

    //-------------------------------------------------------------------------
    /** Возвращает количество ребер в графе */
    int edgesCount() const {
        return _graph_edges.size();
    }

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
                _GraphEdge* edge = (*iter_edges).second;
                // Удалить ребро из графа
                _graph_edges.erase(iter_edges);

                /* Удалить соответствующую запись из списка инцидентных вершин.
                 *
                 * В std::multimap все элементы являются парами std::pair,
                 * упорядоченными по ключу. Найдем в этом множестве первую и
                 * последнюю записи, ключом в которых является вершина start,
                 * тогда все записи между ними относятся к инцидентным ей вершинам.
                 * Найдем среди них запись, в которой конечная вершина принадлежит
                 * удаляемому ребру и удалим эту запись */
                std::pair<
                        typename MultiMapIncidentVertexes::iterator,
                        typename MultiMapIncidentVertexes::iterator
                        > pair_iters_vtxs_range = _graph_incident_vertexes.equal_range(vtx_start);
                typename MultiMapIncidentVertexes::iterator iter_lookup_vtx_end = pair_iters_vtxs_range.first;
                for ( ; iter_lookup_vtx_end != pair_iters_vtxs_range.second &&
                      (*iter_lookup_vtx_end).second != vtx_end;
                      ++iter_lookup_vtx_end )
                { }

                // Если запись успешно найдена, удалить ее
                if ( iter_lookup_vtx_end != pair_iters_vtxs_range.second ) {
                    _graph_incident_vertexes.erase(iter_lookup_vtx_end);
                }

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
    /** Возвращает вес ребра между указанными вершинами, если оно существует, иначе вернет 0 */
    EdgeWeight edgeWeight(VertexID vertex_from, VertexID vertex_to) const {
        EdgeWeight weight = 0;
        _GraphVertex* vtx_start = _findVertex(vertex_from);
        _GraphVertex* vtx_end = _findVertex(vertex_to);
        if (vtx_start && vtx_end) {
            typename MapVertexesEdges::const_iterator iter_edge_lookup =
                    _graph_edges.find( std::make_pair(vtx_start, vtx_end) );
            if (_graph_edges.end() != iter_edge_lookup) {
                _GraphEdge* edge = iter_edge_lookup->second;
                weight = edge->weight();
            }
        }


        return weight;
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
    /** Доп. операция, выводящая на экран структуру графа (список инцидентных вершин) */
    void printGraphStructure() {
        if (0 < _graph_vertexes.size()) {
        /* Последовательно пройти по всем вершинам графа */
        typename MapVertexesWithIds::iterator iter_vtxs = _graph_vertexes.begin();
        for (; _graph_vertexes.end() != iter_vtxs; ++iter_vtxs) {
            /* Для каждой вершины графа найти все исходящие из нее ребра */
            _GraphVertex* next_vtx = (*iter_vtxs).second;
            std::cout << "[" << next_vtx->id() << "] -> ";
            std::pair<typename MultiMapIncidentVertexes::iterator,
                    typename MultiMapIncidentVertexes::iterator> incid_vtxs_start_end_bounds =
                    _graph_incident_vertexes.equal_range(next_vtx);
            /* Пройти по всей группе записей с ключом next_vtx */
            typename MultiMapIncidentVertexes::iterator iter_incid_vtxs =
                    incid_vtxs_start_end_bounds.first;
            for (; incid_vtxs_start_end_bounds.second != iter_incid_vtxs; ++iter_incid_vtxs) {
                _GraphVertex* vtx_to = (*iter_incid_vtxs).second;
                _GraphEdge* edge = _findEdge(next_vtx->id(), vtx_to->id());
                std::cout << vtx_to->id() << "(" << edge->weight() << "), ";
            }
            std::cout << std::endl;
        }
        } else {
            std::cout << "Graph is EMPTY" << std::endl;
        }
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
    void _removeAllIncidentEdges(_GraphVertex* vtx) {
        /* Удалить все ребра, исходящие из указанной вершины */
        /* Выбираем в списке инцидентных вершин те записи, для которых указанная
         * вершина является исходящей */
        std::pair<
                typename MultiMapIncidentVertexes::iterator,
                typename MultiMapIncidentVertexes::iterator
                > pair_iters_equal_range = _graph_incident_vertexes.equal_range(vtx);
        typename MultiMapIncidentVertexes::iterator iter_vtx_start_records = pair_iters_equal_range.first;
        /* Проходим по найденным парам вершин */
        for (;
             iter_vtx_start_records != pair_iters_equal_range.second;
             ++iter_vtx_start_records)
        {
            PairOfVertexes pair_incid_vtxs = (*iter_vtx_start_records);
            /* Удаляем из графа ребро, соответствующее следующей паре вершин */
            _GraphEdge* next_outgoing_edge = _graph_edges[pair_incid_vtxs];
            _graph_edges.erase(pair_incid_vtxs);
            delete next_outgoing_edge;
        }
        /* Стираем пройденные записи из списка инцидентности, которым
         * соответствует исходящая вершина vtx */
        _graph_incident_vertexes.erase(vtx);


        /* Удалить все ребра, входящие в указанную вершину
         *
         * Для этого вновь проходим по списку инцидентных вершин,
         * теперь проверяя те записи, для которых указанная вершина
         * является концевой */
        typename MultiMapIncidentVertexes::iterator iter_lookfor_vtx_end_records = _graph_incident_vertexes.begin();
        for (; _graph_incident_vertexes.end() != iter_lookfor_vtx_end_records;
             ++iter_lookfor_vtx_end_records) {
            PairOfVertexes pair_incid_vtxs = (*iter_lookfor_vtx_end_records);
            /* Если найдена запись, в которой указанная вершина
             * является концевой */
            if (pair_incid_vtxs.second == vtx) {
                /* Удалить соответствующее ей ребро */
                _GraphEdge* next_incoming_edge = _graph_edges[pair_incid_vtxs];
                _graph_edges.erase(pair_incid_vtxs);
                delete next_incoming_edge;

                /* Стереть запись из списка инцидентности
                 *
                 * TODO:
                 * Дальнейшая работа с итератором после удаления элемента
                 * может быть некорректной - всё зависит от конкретной
                 * реализации STL, т.к. подобная ситуация никак не оговорена
                 * стандартом C++ */
                _graph_incident_vertexes.erase(iter_lookfor_vtx_end_records);
            }
        }

        /* Удалить запись в списке вершин графа */
        _graph_vertexes.erase(vtx->id());

        /* Освободить память, отведенную под объект-вершину */
        delete vtx;
    }

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

    //-------------------------------------------------------------------------
    PathDescriptor _createPathDescriptor(_GraphPath* path) {
        PathDescriptor descriptor;
        // Если объект-путь не пустой
        if (path && !path->isEmpty()) {
            // Составить список из идентификаторов его последовательных вершин
            descriptor.path_vertexes.push_back(path->vertexStart()->id());
            typename EdgesPtrList::iterator iter_path_edges = path->edges.begin();
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

    VertexID _next_vacant_vtx_id;


};


//*****************************************************************************
/** Часть консольной программы симулятора, моделирующая работу алгоритма Дейстры для всего графа
 *
 * Работа симулятора состоит в последовательном запуске алгоритма Дейкстры
 * для каждой из вершин графа с целью поиска всех кратчайших маршрутов между
 * всеми вершинами графа. */
template <class TData = int>
class DjkstraGraphSimulator {
public:
    DjkstraGraphSimulator( const OrientedGraph<TData> * graph )
        : _graph(graph)
    {}

    //-------------------------------------------------------------------------
    void run() {
        CLEAR_SCREEN();

        if (!_graph->isEmpty()) {
            std::list< typename OrientedGraph<TData>::PathDescriptor > eccentricities_list;

            typename OrientedGraph<TData>::VertexesList all_vtxs = _graph->vertexes();
            typename OrientedGraph<TData>::VertexesList::iterator iter_all_vtxs = all_vtxs.begin();
            /* Последовательно запускать алгоритм Декйкстры для каждой вершины графа,
             * запоминая  найденные эксцентриситеты */
            for (; all_vtxs.end() != iter_all_vtxs; ++iter_all_vtxs) {
                typename OrientedGraph<TData>::VertexID next_vid = (*iter_all_vtxs);
                VertexSimulator vertex_simulator(_graph, next_vid);
                vertex_simulator.run();

                /* Отобразить промежуточные результаты для очередной вершины */
                typename OrientedGraph<TData>::PathDescriptor next_eccentricity =
                        vertex_simulator.getEccentricity();
                if (!next_eccentricity.isEmpty()) {
                    /* Запомнить найденный эксцентриситет */
                    eccentricities_list.push_back(next_eccentricity);
                    std::cout << "Next eccentricity: "
                              << std::endl;
                    next_eccentricity.print();
                    std::cout << std::endl;
                    PAUSE();
                }
                /* Если не найдено ни одного пути из вершины */
                else {
                    CLEAR_SCREEN();
                    std::cout << "No routes found from vertex [" << next_vid << "]"
                              << std::endl;
                    PAUSE();
                }
            }

            /* Наименьший эксцентриситет в графе - его радиус */
            typename std::list<typename OrientedGraph<TData>::PathDescriptor>::iterator
                    iter_eccentricities = eccentricities_list.begin();
            typename OrientedGraph<TData>::PathDescriptor radius;
            for (; eccentricities_list.end() != iter_eccentricities; ++iter_eccentricities) {
                typename OrientedGraph<TData>::PathDescriptor next_eccentricity = (*iter_eccentricities);
                if (radius.isEmpty() ||
                        next_eccentricity.weight < radius.weight) {
                    radius = next_eccentricity;
                }
            }

            CLEAR_SCREEN();
            std::cout << "Finished. Graph radius: "
                      << std::endl;
            radius.print();
            std::cout << std::endl;
        } else {
            std::cout << "Graph is EMPTY" << std::endl;
        }

        PAUSE();
    }

    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------


private:
    /*****************************************************************************/
    /** Часть консольной программы симулятора, моделирующая работу алгоритма Дейстры для конкретной исходной вершины */
    class VertexSimulator {

        static const unsigned int Infinite = INT_MAX;


    public:
        VertexSimulator( const OrientedGraph<TData> * graph,
                                            typename OrientedGraph<TData>::VertexID start_vertex_id )
            : _graph(graph),
              _vtx_start_id(start_vertex_id),
              _is_finished(false),
              _last_changed_vertex(0)
        {
            _reset();
        }

        //-------------------------------------------------------------------------
        bool isFinished() const {
            return _is_finished;
        }

        //-------------------------------------------------------------------------
        void run() {
            while(!isFinished()) {
                _displayStats();
                getchar();
                _stepOver();
            }
        }
        //-------------------------------------------------------------------------
        /** Возвращает путь с наибольшей длиной */
        typename OrientedGraph<TData>::PathDescriptor getEccentricity() const {
            typename OrientedGraph<TData>::PathDescriptor path;
            /* Если выполнение алгоритма было завершено */
            if (isFinished()) {
                /* Если из исходной вершины найден хотя бы один путь,
                 * список завершенных вершин будет содержать как минимум
                 * две вершины.
                 * В противном случае исходная вершина будет единственной
                 * посещенной. */
                if (1 < _vtxs_completed.size()) {
                    Distance dist_max = 0;
                    typename OrientedGraph<TData>::VertexID vtx_maxdist = 0;
                    typename MapDjkstraVertexes::const_iterator iter_end_vtxs = _vtxs_completed.begin();
                    /* Найти вершину с наибольшим расстоянием до нее */
                    for (; _vtxs_completed.end() != iter_end_vtxs; ++iter_end_vtxs) {
                        Distance next_dist = (*iter_end_vtxs).second;
                        if (dist_max < next_dist) {
                            dist_max = next_dist;
                            vtx_maxdist = (*iter_end_vtxs).first;
                        }
                    }
                    path.weight = dist_max;

                    /* Выстроить маршрут до найденной вершины */
                    typename OrientedGraph<TData>::VertexID vtx_end = vtx_maxdist;
                    path.prependVertex(vtx_end);
                    /* Перейти к последней вершине пути */
                    typename MapPathBreadcrumbs::const_iterator iter_path_breadcrumbs = _path_breadcrumbs.find(vtx_end);
                    /* Двигаться последовательно от последней вершины к предыдущим,
                     * пока не встретится исходная */
                    while(_path_breadcrumbs.end() != iter_path_breadcrumbs) {
                        typename OrientedGraph<TData>::VertexID vtx_prev = (*iter_path_breadcrumbs).second;
                        path.prependVertex(vtx_prev);
                        /* Перейти к предыдущей вершине в пути */
                        iter_path_breadcrumbs = _path_breadcrumbs.find(vtx_prev);
                    }
                }
            }


            return path;
        }

        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------


    private:
        typedef unsigned int Distance;
        typedef std::map <typename OrientedGraph<TData>::VertexID /* ID вершины */,
            Distance /* значение числовой метки */>
            MapDjkstraVertexes;


        const OrientedGraph<TData> * _graph;

        /** ID исходной вершины, от которой строятся кратчайшие маршруты */
        typename OrientedGraph<TData>::VertexID _vtx_start_id;

        /** Множество "еще не посещенных" вершин */
         MapDjkstraVertexes _vtxs_non_visited;

        /** Множество посещенных вершин
         *
         * В этот список вершины попадают после того, как до них
         * окончательно найден кратчайший путь */
        MapDjkstraVertexes _vtxs_completed;

        typedef std::map < typename OrientedGraph<TData>::VertexID /* vtx_to */,
            typename OrientedGraph<TData>::VertexID /* vtx_previous */ >
            MapPathBreadcrumbs;
        /** Вершины-ключи данного множества являются конечными точками соответствующих
         * кратчайших путей из вершины vtx_start. Значения, хранящиеся в данном множестве,
         * представляют предпоследние точки этих кратчайших маршрутов: таким образом,
         * зная для каждой вершины предшествующую ей на пути вершину, методом последовательного
         * обхода можно восстановить любой из найденных путей полностью */
         MapPathBreadcrumbs _path_breadcrumbs;


        /** Вершина, посещаемая на текущем шаге алгоритма */
        typename OrientedGraph<TData>::VertexID _curr_step_vertex;


        /** Вершины-соседи, в которые идут ребра из рассматриваемой на текущем шаге
         * алгоритма вершины */
        typename OrientedGraph<TData>::VertexesList _curr_sibling_vertexes;


        /** Флаг-индикатор завершения работы алгоритма */
        bool _is_finished;


        /** Левый отступ для выводимого текста в консоли */
        unsigned int _output_left_margin;


        /** ID вершины, затронутой последним шагом алгоритма.
         *
         * Данная информация может быть удобна для визуального
         * выделения последней измененной вершины на экране. */
        typename OrientedGraph<TData>::VertexID _last_changed_vertex;


        std::stack<WORD> _console_text_attributes;


        //-------------------------------------------------------------------------
        /** Переводит алгоритм к следующему шагу
         *
         * Каждый шаг программы - это либо переход к следующей непосещенной вершине,
         * либо попытка вычисления новой метки для очередной вершины-соседа текущей
         * посещаемой вершины */
        void _stepOver() {
            /* Если еще остались непросмотренные соседи текущей вершины */
            if ( _nextSiblingVertex() ) {
                /* Взять следующюю вершину-соседа */
                typename OrientedGraph<TData>::VertexID vtxid_next_sibling = _takeNextSiblingVertex();

                /* Рассчитать длину пути до вершины-соседа через текущую вершину */
                Distance curr_distance_to_sibling = _vtxs_non_visited[vtxid_next_sibling];
                typename OrientedGraph<TData>::EdgeWeight edge_weight = _graph->edgeWeight(_curr_step_vertex, vtxid_next_sibling);
                Distance new_distance_to_sibling = _vtxs_non_visited[_curr_step_vertex] + edge_weight;
                /* Если длина пути меньше, запомнить новый маршрут */
                if (new_distance_to_sibling < curr_distance_to_sibling) {
                    _vtxs_non_visited[vtxid_next_sibling] = new_distance_to_sibling;
                    this->_path_breadcrumbs[vtxid_next_sibling] = _curr_step_vertex;
                }

                _last_changed_vertex = vtxid_next_sibling;
            }
            /* Перейти к следующей вершине */
            else {
                /* Пометить текущую вершину как посещенную, запомнив длину пути до нее */
                Distance distance = _vtxs_non_visited[_curr_step_vertex];
                _vtxs_non_visited.erase(_curr_step_vertex);
                _vtxs_completed.insert( std::make_pair(_curr_step_vertex, distance) );
                _last_changed_vertex = _curr_step_vertex;

                /* Если еще остались непосещенные вершины */
                if ( !_vtxs_non_visited.empty() ) {
                    /* Найти следующую вершину с наименьшим расстоянием до нее */
                    Distance min_dist = Infinite;
                    typename OrientedGraph<TData>::VertexID next_vtxid = 0;
                    typename MapDjkstraVertexes::iterator iter_vtxs = _vtxs_non_visited.begin();
                    for (; _vtxs_non_visited.end() != iter_vtxs; ++iter_vtxs) {
                        typename OrientedGraph<TData>::VertexID vid_next = (*iter_vtxs).first;
                        Distance dist_next = (*iter_vtxs).second;
                        if (dist_next < min_dist) {
                            min_dist = dist_next;
                            next_vtxid = vid_next;
                        }
                    }
                    /* Если найдена вершина с меткой, отличной от бесконечности */
                    if (next_vtxid) {
                        /* Перейти к рассмотрению найденной вершины и её соседей */
                        _curr_step_vertex = next_vtxid;
                        /* Выбрать для обработки те вершины, которые еще не были посещены */
                        typename OrientedGraph<TData>::VertexesList siblings = _graph->vertexOutSiblings(_curr_step_vertex);
                        typename OrientedGraph<TData>::VertexesList::iterator iter_siblings = siblings.begin();
                        for (; siblings.end() != iter_siblings; ++iter_siblings) {
                            typename OrientedGraph<TData>::VertexID vtxid_next_sibling = (*iter_siblings);
                            typename MapDjkstraVertexes::iterator iter_vtx_visited_lookup =
                                    _vtxs_completed.find(vtxid_next_sibling);
                            /* Если до этой вершины еще не было найдено кратчайшего пути
                         * (она еще не отмечена посещенной) */
                            if ( _vtxs_completed.end() == iter_vtx_visited_lookup ) {
                                _curr_sibling_vertexes.push_back(vtxid_next_sibling);
                            }
                        }
                    }
                    /* Иначе до оставшихся вершин больше не существует маршрутов
                     * из исходной вершины. Алгоритм завершен. */
                    else {
                        _is_finished = true;
                    }
                }
                /* Иначе алгоритм завершен */
                else {
                    _is_finished = true;
                }
            }
        }

        //-------------------------------------------------------------------------
        void _continueTillEnd() {
            while (!isFinished()) {
                _stepOver();
            }
        }

        //-------------------------------------------------------------------------
        unsigned int _leftMarginForConsoleOutput() const {
            return _output_left_margin;
        }

        //-------------------------------------------------------------------------
        void _setLeftMarginForConsoleOutput(unsigned int x) {
            _output_left_margin = x;
        }

        //-------------------------------------------------------------------------
        void _setConsoleTextAttributes(WORD attributes) {
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
            CONSOLE_SCREEN_BUFFER_INFO scr_buff_info;
            GetConsoleScreenBufferInfo(hConsole, &scr_buff_info);
            /* Сохранить предыдущие значения атрибутов */
            _console_text_attributes.push(scr_buff_info.wAttributes);

            /* Установить заданные атрибуты */
            SetConsoleTextAttribute(hConsole, attributes);
        }

        //-------------------------------------------------------------------------
        void _restoreConsoleTextAttributes() {
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
            SetConsoleTextAttribute(hConsole, _console_text_attributes.top());
            _console_text_attributes.pop();
        }

        //-------------------------------------------------------------------------
        void _gotoNextConsoleLine() {
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
            CONSOLE_SCREEN_BUFFER_INFO scr_buff_info;
            GetConsoleScreenBufferInfo(hConsole, &scr_buff_info);
            COORD cursor_pos = scr_buff_info.dwCursorPosition;
            cursor_pos.X = _leftMarginForConsoleOutput();
            cursor_pos.Y += 1;

            SetConsoleCursorPosition(hConsole, cursor_pos);
        }

        //-------------------------------------------------------------------------
        void __printDistance(Distance dist) {
            if (Infinite == dist) {
                std::cout << "INF";
            } else { std::cout << dist; }
        }
        //-------------------------------------------------------------------------
        void __printFormattedVertexWithDistance(typename OrientedGraph<TData>::VertexID vid,
                                                Distance dist) {
            /* Визуально выделить вершину, затронутую последним шагом алгоритма */
            if (vid == _last_changed_vertex) {
                _setConsoleTextAttributes(__textAttributesForLastChangedVertex());
            }

            std::cout << std::setw(3)
                      << std::setfill(' ')
                      << std::setiosflags(std::ios::right)
                      << vid << " [ ";
            __printDistance(dist);
            std::cout << " ] ";

            if (vid == _last_changed_vertex) {
                _restoreConsoleTextAttributes();
            }
        }

        //-------------------------------------------------------------------------
        void __displayVertexesList() {
            /* Вывести сперва список непосещенных вершин */
            typename MapDjkstraVertexes::const_iterator iter_graph_vtxs =
                    _vtxs_non_visited.begin();
            for (; _vtxs_non_visited.end() != iter_graph_vtxs; ++iter_graph_vtxs) {
                typename OrientedGraph<TData>::VertexID vid = (*iter_graph_vtxs).first;
                Distance dist = (*iter_graph_vtxs).second;
                __printFormattedVertexWithDistance(vid, dist);
                _gotoNextConsoleLine();
            }
            std::cout << "--visited:--";
            _gotoNextConsoleLine();

            /* Затем вывести список посещенных вершин */
            _setConsoleTextAttributes(__textAttributesForVisitedVertex());
            iter_graph_vtxs = _vtxs_completed.begin();
            for (; _vtxs_completed.end() != iter_graph_vtxs; ++iter_graph_vtxs) {
                typename OrientedGraph<TData>::VertexID vid = (*iter_graph_vtxs).first;
                Distance dist = (*iter_graph_vtxs).second;
                __printFormattedVertexWithDistance(vid, dist);
                _gotoNextConsoleLine();
            }
            _restoreConsoleTextAttributes();
        }

        //-------------------------------------------------------------------------
        void __displayCurrentVertexSiblings() {
            std::cout << "CURRENT VERTEX: " << _curr_step_vertex;
            _gotoNextConsoleLine();
            std::cout << "SIBLINGS: ";
            _gotoNextConsoleLine();
            typename OrientedGraph<TData>::VertexID next_vid = _nextSiblingVertex();
            typename OrientedGraph<TData>::VertexesList::iterator iter_siblings =
                    _curr_sibling_vertexes.begin();
            for (; _curr_sibling_vertexes.end() != iter_siblings; ++iter_siblings) {
                typename OrientedGraph<TData>::VertexID vid = (*iter_siblings);
                Distance dist = _vtxs_non_visited[vid];
                /* Визуально пометить вершину, которая будет обработана следующей */
                if (next_vid == vid) {
                    std::cout << "->";
                }
                __printFormattedVertexWithDistance(vid, dist);
                _gotoNextConsoleLine();
            }
        }

        //-------------------------------------------------------------------------
        void __displayProposedAction() {
            typename OrientedGraph<TData>::VertexID next_vid = _nextSiblingVertex();
            if (next_vid) {
                Distance curr_dist = _vtxs_non_visited[next_vid];
                Distance new_dist = _vtxs_non_visited[_curr_step_vertex] +
                        _graph->edgeWeight(_curr_step_vertex, next_vid);

                std::cout << "Current distance to [" << next_vid << "] is ";
                __printDistance(curr_dist);
                _gotoNextConsoleLine();
                std::cout << "Distance through [" << next_vid << "] is ";
                __printDistance(new_dist);
            } else {
                std::cout << "Done with [" << _curr_step_vertex << "]";
            }
            _gotoNextConsoleLine();
        }
        //-------------------------------------------------------------------------
        void _displayStats() {
            CLEAR_SCREEN();
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

            /* Вывести список вершин графа и текущих расстояний до них */
            COORD firstColULCornerCoord = _s_cFirstColumnULCornerCoord();
            SetConsoleCursorPosition(hConsole, firstColULCornerCoord);
            _setLeftMarginForConsoleOutput(firstColULCornerCoord.X);
            __displayVertexesList();


            /* Вывести текущую вершину и список ее соседей,
             * которые еще не обработаны алгоритмом */
            COORD secondColULCornerCoord = _s_cSecondColumnULCornerCoord();
            SetConsoleCursorPosition(hConsole, secondColULCornerCoord);
            _setLeftMarginForConsoleOutput(secondColULCornerCoord.X);
            __displayCurrentVertexSiblings();


            /* Вывести информацию о следующей вершине,
             * которая будет обработана алгоритмом */
            COORD thirdColULCornerCoord = _s_cThirdColumnULCornerCoord();
            SetConsoleCursorPosition(hConsole, thirdColULCornerCoord);
            _setLeftMarginForConsoleOutput(thirdColULCornerCoord.X);
            __displayProposedAction();
        }
        //-------------------------------------------------------------------------
        /** Возвращает ход алгоритма к начальному шагу */
        void _reset() {
            /* Очистить список найденных маршрутов */
            _path_breadcrumbs.clear();

            /* Пометить все вершины графа как непосещенные,
             * присвоив им числовые метки, равные бесконечности;
             * стартовой вершине назначить метку 0. */
            _vtxs_completed.clear();
            _vtxs_non_visited.clear();
            _vtxs_non_visited.insert( std::make_pair(_vtx_start_id, 0) );
            typename OrientedGraph<TData>::VertexesList all_vertexes = _graph->vertexes();
            typename OrientedGraph<TData>::VertexesList::iterator iter_vtxs = all_vertexes.begin();
            /* Назначить всем вершинам метки "бесконечность", кроме стартовой */
            for (; all_vertexes.end() != iter_vtxs; ++iter_vtxs) {
                typename OrientedGraph<TData>::VertexID vid = (*iter_vtxs);
                if (vid != _vtx_start_id) {
                    _vtxs_non_visited[vid] = Infinite;
                }
            }

            /* Назначить текущей вершиной шага стартовую вершину
             * и подготовить список осматриваемых вершин-соседей. */
            _curr_step_vertex = _vtx_start_id;
            _curr_sibling_vertexes.clear();
            _curr_sibling_vertexes = _graph->vertexOutSiblings(_vtx_start_id);

            /* Сбросить флаг завершенности алгоритма */
            _is_finished = false;
        }

        //-------------------------------------------------------------------------
        /** Возвращает ID вершины, которая будет обработана на следующем шаге алгоритма
         *
         * Если вершин-соседей для обработки не осталось, вернет 0. */
        typename OrientedGraph<TData>::VertexID _nextSiblingVertex() const {
            typename OrientedGraph<TData>::VertexID next_vtx = 0;
            if (!_curr_sibling_vertexes.empty()) {
                next_vtx = _curr_sibling_vertexes.front();
            }


            return next_vtx;
        }

        //-------------------------------------------------------------------------
        /** Возвращает ID следующей по порядку вершины-соседа с удалением её из списка */
        typename OrientedGraph<TData>::VertexID _takeNextSiblingVertex() {
            typename OrientedGraph<TData>::VertexID next_vid = _nextSiblingVertex();
            if(next_vid) {
                _curr_sibling_vertexes.pop_front();
            }


            return next_vid;
        }

        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        /** Возвращает координаты левого верхнего угла первой колонки вывода в консоли */
        static COORD _s_cFirstColumnULCornerCoord() {
            COORD coord;
            coord.X = 1;
            coord.Y = 1;


            return coord;
        }

        //-------------------------------------------------------------------------
        /** Возвращает координаты левого верхнего угла второй колонки вывода в консоли */
        static COORD _s_cSecondColumnULCornerCoord() {
            COORD coord;
            coord.X = 17;
            coord.Y = 1;


            return coord;
        }

        //-------------------------------------------------------------------------
        /** Возвращает координаты левого верхнего угла третьей колонки вывода в консоли */
        static COORD _s_cThirdColumnULCornerCoord() {
            COORD coord;
            coord.X = 40;
            coord.Y = 1;


            return coord;
        }

        //-------------------------------------------------------------------------
        static WORD __textAttributesForLastChangedVertex() {
            /* Ярко-жёлтый текст */
            return 6 | FOREGROUND_INTENSITY;
        }

        //-------------------------------------------------------------------------
        static WORD __textAttributesForVisitedVertex() {
            /* Серый текст */
            return FOREGROUND_INTENSITY;
        }

        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
        //-------------------------------------------------------------------------
    };


    const OrientedGraph<TData> * _graph;
};




#endif // ORIENTEDGRAPH_H
