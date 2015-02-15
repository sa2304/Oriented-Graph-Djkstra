#ifndef MAINMENU_H
#define MAINMENU_H

#include <string>

#include "IMenu.h"
#include "orientedgraph.h"

class MainMenu : public IMenu {
	static const std::string MenuItemText_CreateVertex;
	static const std::string MenuItemText_DeleteVertex;
	static const std::string MenuItemText_CreateEdge;
	static const std::string MenuItemText_DeleteEdge;
    static const std::string MenuItemText_PrintGraphStructure;
    static const std::string MenuItemText_DjkstraSimulation;
    static const std::string MenuItemText_PrintVertexesCount;
    static const std::string MenuItemText_PrintEdgesCount;
    static const std::string MenuItemText_CheckIfEdgeExists;
    static const std::string MenuItemText_SetEdgeData;
	
    /*************************************************************************/
	class TextAction : public IMenuAction {
	public:
		TextAction(std::string title);
		std::string text() const;

	private:
		std::string _text;
	};

    /*************************************************************************/

public:
	MainMenu();

	void run();

private:
	void _initActions();
    void _dbgInitGraph();

	int _askVertexId();
	int _askStartVertexId();
	int _askEndVertexId();
	int _askEdgeWeight();
    long long _promptNumber(unsigned int max_digits_count);


	OrientedGraph<int> _graph;

	TextAction* _action_create_vertex;
	TextAction* _action_remove_vertex;
	TextAction* _action_create_edge;
	TextAction* _action_remove_edge;
	TextAction* _action_print_graph_structure;
    TextAction* _action_djsktra_simulation;
    TextAction* _action_print_vertexes_count;
    TextAction* _action_print_edges_count;
    TextAction* _action_check_if_edge_exists;
    TextAction* _action_set_edge_data;


};


#endif
