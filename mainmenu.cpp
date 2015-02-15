#include "mainmenu.h"

const std::string MainMenu::MenuItemText_CreateVertex = "Create vertex";
const std::string MainMenu::MenuItemText_DeleteVertex = "Remove vertex";
const std::string MainMenu::MenuItemText_CreateEdge = "Create edge";
const std::string MainMenu::MenuItemText_DeleteEdge = "Remove edge";
const std::string MainMenu::MenuItemText_PrintGraphStructure = "Print graph structure";
const std::string MainMenu::MenuItemText_DjkstraSimulation = "Djkstra simulation (Find radius)";
const std::string MainMenu::MenuItemText_PrintVertexesCount = "Vertexes count";
const std::string MainMenu::MenuItemText_PrintEdgesCount = "Edges count";
const std::string MainMenu::MenuItemText_CheckIfEdgeExists = "Check if edge exists";
const std::string MainMenu::MenuItemText_SetEdgeData = "Set edge data";

MainMenu::MainMenu() 
: _action_create_vertex(NULL),
	_action_remove_vertex(NULL),
	_action_create_edge(NULL),
	_action_remove_edge(NULL),
    _action_print_graph_structure(NULL),
    _action_djsktra_simulation(NULL),
    _action_print_vertexes_count(NULL),
    _action_print_edges_count(NULL),
    _action_check_if_edge_exists(NULL),
    _action_set_edge_data(NULL)
{
	_initActions();


//    _dbgInitGraph();
}

//-----------------------------------------------------------------------------
void MainMenu::_initActions() {
	_action_create_vertex = new TextAction(MenuItemText_CreateVertex);
	addAction(_action_create_vertex);

	_action_remove_vertex = new TextAction(MenuItemText_DeleteVertex);
	addAction(_action_remove_vertex);

    _action_print_vertexes_count = new TextAction(MenuItemText_PrintVertexesCount);
    addAction(_action_print_vertexes_count);

	_action_create_edge = new TextAction(MenuItemText_CreateEdge);
	addAction(_action_create_edge);

	_action_remove_edge = new TextAction(MenuItemText_DeleteEdge);
	addAction(_action_remove_edge);

    _action_print_edges_count = new TextAction(MenuItemText_PrintEdgesCount);
    addAction(_action_print_edges_count);

    _action_check_if_edge_exists = new TextAction(MenuItemText_CheckIfEdgeExists);
    addAction(_action_check_if_edge_exists);

    _action_set_edge_data = new TextAction(MenuItemText_SetEdgeData);
    addAction(_action_set_edge_data);

	_action_print_graph_structure = new TextAction(MenuItemText_PrintGraphStructure);
	addAction(_action_print_graph_structure);

    _action_djsktra_simulation = new TextAction(MenuItemText_DjkstraSimulation);
    addAction(_action_djsktra_simulation);
}

//-----------------------------------------------------------------------------
void MainMenu::_dbgInitGraph() {
    for (int i = 0; i < 5; ++i) {
        _graph.createVertex();
    }

    _graph.createEdge(1, 2, 10);
    _graph.createEdge(2, 3, 15);
}

//-----------------------------------------------------------------------------
MainMenu::TextAction::TextAction(std::string title)
: _text(title)
{
}

//-----------------------------------------------------------------------------
std::string MainMenu::TextAction::text() const {
	return _text;
}

//-----------------------------------------------------------------------------
void MainMenu::run() {
	while (!isClosedByUser()) {
		IMenuAction* selected_action = exec();
		if (selected_action == _action_create_vertex) {
			CLEAR_SCREEN();
			int vtxid = _graph.createVertex();
			std::cout << vtxid << std::endl;
			PAUSE();
		} 
		else if (selected_action == _action_remove_vertex) {
			CLEAR_SCREEN();
			int vtxid = _askVertexId();
			std::cout << _graph.removeVertex(vtxid) << std::endl;
			PAUSE();
		}
		else if (selected_action == _action_create_edge) {
			CLEAR_SCREEN();
			int vtxid_start = _askStartVertexId();
			CLEAR_SCREEN();
			int vtxid_end = _askEndVertexId();
			CLEAR_SCREEN();
			int edge_weight = _askEdgeWeight();
			CLEAR_SCREEN();
			std::cout << _graph.createEdge(vtxid_start, vtxid_end, edge_weight) << std::endl;
			PAUSE();
		}
		else if (selected_action == _action_remove_edge) {
			CLEAR_SCREEN();
			int vtxid_start = _askStartVertexId();
			CLEAR_SCREEN();
			int vtxid_end = _askEndVertexId();
			CLEAR_SCREEN();
			std::cout << _graph.deleteEdge(vtxid_start, vtxid_end) << std::endl;
			PAUSE();
		}
        else if (selected_action == _action_print_graph_structure) {
            CLEAR_SCREEN();
            _graph.printGraphStructure();
            PAUSE();
        }
        else if (selected_action == _action_djsktra_simulation) {
            DjkstraGraphSimulator<int> djkstra_simulator(&_graph);
            djkstra_simulator.run();
        }
        else if (selected_action == _action_print_vertexes_count) {
            CLEAR_SCREEN();
            std::cout << _graph.vertexesCount() << std::endl;
            PAUSE();
        }
        else if (selected_action == _action_print_edges_count) {
            CLEAR_SCREEN();
            std::cout << _graph.edgesCount() << std::endl;
            PAUSE();
        }
        else if (selected_action == _action_check_if_edge_exists) {
            CLEAR_SCREEN();
            int vtxid_start = _askStartVertexId();
            CLEAR_SCREEN();
            int vtxid_end = _askEndVertexId();
            CLEAR_SCREEN();
            std::cout << _graph.hasEdge(vtxid_start, vtxid_end);
            PAUSE();
        }
        else if (selected_action == _action_set_edge_data) {
            CLEAR_SCREEN();
            int vtxid_start = _askStartVertexId();
            CLEAR_SCREEN();
            int vtxid_end = _askEndVertexId();
            CLEAR_SCREEN();
            int data = _promptNumber(5);
            CLEAR_SCREEN();
            std::cout << _graph.setEdgeData(vtxid_start, vtxid_end, data);
            PAUSE();
        }
	}
}
//-----------------------------------------------------------------------------
int MainMenu::_askVertexId() {
    CLEAR_SCREEN();
    std::cout << "Enter vertex ID: ";
    return _promptNumber(3);
}

//-----------------------------------------------------------------------------
int MainMenu::_askStartVertexId() {
    CLEAR_SCREEN();
    std::cout << "Enter start vertex ID: ";
    return _promptNumber(3);
}

//-----------------------------------------------------------------------------
int MainMenu::_askEndVertexId() {
    CLEAR_SCREEN();
    std::cout << "Enter end vertex ID: ";
    return _promptNumber(3);
}

//-----------------------------------------------------------------------------
int MainMenu::_askEdgeWeight() {
    CLEAR_SCREEN();
    std::cout << "Enter edge weight: ";
    return _promptNumber(3);
}

//-----------------------------------------------------------------------------
long long MainMenu::_promptNumber(unsigned int max_digits_count = 8) {
    long long number = 0;
    HANDLE hConsole = GetStdHandle(STD_INPUT_HANDLE);
    DWORD num_read;
    INPUT_RECORD irInBuf[128];

    for (int i = 0; i < max_digits_count; ++i) {
        std::cout << "_";
    }
    for (int i = 0; i < max_digits_count; ++i) {
        std::cout << "\b";
    }

    int digits_entered_count = 0;
    while (true) {
        ReadConsoleInput(hConsole, irInBuf, 128, &num_read);
        for(int i = 0; i < num_read; ++i) {
            INPUT_RECORD next_record = irInBuf[i];
            KEY_EVENT_RECORD* key_evt = NULL;
            switch(next_record.EventType) {
            case KEY_EVENT:
                key_evt = &next_record.Event.KeyEvent;
                if (key_evt->bKeyDown) {
                    if (VK_BACK == key_evt->wVirtualKeyCode) {
                        if (0 < digits_entered_count) {
                            std::cout << "\b_\b";
                            number /= 10;
                            --digits_entered_count;
                        }
                    }
                    else if (digits_entered_count < max_digits_count &&
                             key_evt->wVirtualKeyCode <= 0x39 &&
                             0x30 <= key_evt->wVirtualKeyCode) {
                        WORD repeat_count = key_evt->wRepeatCount;
                        while (0 < repeat_count--) {
                            int digit = (key_evt->wVirtualKeyCode - 0x30);
                            number *= 10;
                            number += digit;
                            std::cout << digit;
                            ++digits_entered_count;
                        }
                    }
                    else if (VK_RETURN == key_evt->wVirtualKeyCode) {
                        std::cout << std::endl;
                        return number;
                    }
                }
                break;
            }
        }
    }
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
