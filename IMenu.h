#pragma once
#include <vector>
#include <Windows.h>
#include <WinCon.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "IMenuAction.h"
#include "imenuhotkeyhandler.h"

class IMenu
{
public:
	IMenu(void);
    virtual ~IMenu(void);

    std::string name() const;
    void setName(std::string name);
    std::string header() const;
    void setHeader(std::string header);
    std::string footer() const;
    void setFooter(std::string footer);
    void addAction(IMenuAction* action);
    void removeAction(IMenuAction* action);
    void removeAction(int index);
    void removeAllActions();
	int actionsCount() const;
    IMenuAction* actionAt(int idx) const;
    int indexOfAction(IMenuAction* action) const;
    IMenuAction* selectedAction() const;
    int selectedIndex() const;
    void setSelectedAction(int index);
    void selectNextMenuAction();
    void selectPreviousMenuAction();
    void clear();

    bool isClosedByUser() const;
    void setClosedByUser(bool state);

    static void clearConsole();
    static void moveConsoleCursorInTopLeftCorner();

    static const int CLOSED_BY_USER = -1;
    static const int NO_SELECTION = -1;

protected:
    IMenuAction *exec();
    IMenuHotkeysHandler* hotkeysHandler() const;
    void setHotkeysHandler(IMenuHotkeysHandler* handler);
    virtual std::string defaultMenuHeader() const;
    virtual std::string defaultMenuFooter() const;
    virtual void showMenuHelp() const;
    virtual std::string menuHelpText() const;

private:
    std::string _name;
    std::string _header;
    std::string _footer;
    std::vector<IMenuAction*> _actions;
    int _selected_action_idx;
    int _topmost_visible_action_idx;
    int _visible_actions_count; ///< Количество пунктов меню, которые позволяют отобразить текущие размеры окна
    DWORD _line_height;
    int _menu_page_size; ///< Количество пунктов меню, которые "перескочит" выделение при нажатии PG_UP/PG_DN
    IMenuHotkeysHandler* _hotkeys_handler;
    bool _is_closed_by_user;
    int _old_console_text_attributes;


    void _repaint();
	void _printMenuActions() const;
    void _gotoPreviousMenuPage();
    void _gotoNextMenuPage();

    bool _isHiddenMenuEntriesBeforeTop() const;
    int _hiddenMenuEntriesBeforeTopCount() const;
    bool _isHiddenMenuEntriesAfterBottom() const;
    int _hiddenMenuEntriesAfterBottomCount() const;

    void _resize(COORD size);

    void _handleHotkey(WORD key_code, IMenuAction* action);

    bool _isHelpTextProvided() const;

    static CONSOLE_FONT_INFOEX _getConsoleFontForSelectedAction();

    static int getConsoleTextAttributesForMenu();
    static int getConsoleTextAttributesForSelectedItem();

};

