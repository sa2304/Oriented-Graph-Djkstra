#include "IMenu.h"

#ifdef DEBUG

#include <stdio.h>

#endif

IMenu::IMenu(void)
    : _selected_action_idx(NO_SELECTION),
      _topmost_visible_action_idx(0),
      _hotkeys_handler(NULL),
      _is_closed_by_user(false),
      _old_console_text_attributes(0)
{
    /* Рассчитать количество пунктов меню,
     * которые позволяет отобразить текущие размеры окна */
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    CONSOLE_SCREEN_BUFFER_INFO screen_info;
    GetConsoleScreenBufferInfo(hConsole, &screen_info);

    CONSOLE_FONT_INFO font_info;
    GetCurrentConsoleFont(hConsole, false, &font_info);
    _line_height = font_info.dwFontSize.Y;

    _resize(screen_info.dwSize);

    /* Сохранить предыдущие атрибуты текста в консоли */
    CONSOLE_SCREEN_BUFFER_INFO console_info;
    GetConsoleScreenBufferInfo(hConsole, &console_info);
    _old_console_text_attributes = console_info.wAttributes;

    /* Установить атрибуты текста для меню */
    SetConsoleTextAttribute(hConsole, getConsoleTextAttributesForMenu());

    /* Установить надписи по умолчанию */
    setHeader(defaultMenuHeader());
    setFooter(defaultMenuFooter());
}

//-----------------------------------------------------------------------------
IMenu::~IMenu(void)
{
    /* Восстановить предыдущие атрибуты текста в консоли */
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, _old_console_text_attributes);
}

//-----------------------------------------------------------------------------
IMenuAction* IMenu::exec() {
    clearConsole();
    _repaint();

    /* Сохранить предыдущий режим консоли */
    HANDLE hConsole = GetStdHandle(STD_INPUT_HANDLE);
    DWORD old_console_mode;
    GetConsoleMode(hConsole, &old_console_mode);

    /* Активировать обработку оконных событий и событий мыши */
    SetConsoleMode(hConsole, ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT);

    DWORD num_read;
    INPUT_RECORD irInBuf[128];
	/* Ожидание выбора пункта меню */
	while(true) {
        /* Прочитать события */
        ReadConsoleInput(hConsole, irInBuf, 128, &num_read);

        /* Обработать события */
        for(int i = 0; i < num_read; ++i) {
            INPUT_RECORD next_record = irInBuf[i];
            KEY_EVENT_RECORD* key_evt = NULL;
            MOUSE_EVENT_RECORD* mouse_evt = NULL;
            WINDOW_BUFFER_SIZE_RECORD* win_evt = NULL;

            switch(next_record.EventType) {
            case KEY_EVENT:
                key_evt = &next_record.Event.KeyEvent;
                if(key_evt->bKeyDown) {
                    switch(key_evt->wVirtualKeyCode) {
                    case VK_F1:
                        showMenuHelp();
                        clearConsole();
                        _repaint();
                        break;
                    case VK_UP:
                        selectPreviousMenuAction();
                        _repaint();
                        break;
                    case VK_DOWN:
                        selectNextMenuAction();
                        _repaint();
                        break;
                    case VK_RETURN:
                        /* При нажатии Enter менню завершает работу, возвращая индекс
                         * выбранного пункта меню */
                        SetConsoleMode(hConsole, old_console_mode);
                        return selectedAction();
                        break;
                    case VK_ESCAPE:
                        /* Нажатие клавиши Escape завершает работу меню */
                        SetConsoleMode(hConsole, old_console_mode);
                        setClosedByUser(true);
                        return NULL;
                        break;
                    case VK_PRIOR:
                        _gotoPreviousMenuPage();
                        _repaint();
                        break;
                    case VK_NEXT:
                        _gotoNextMenuPage();
                        _repaint();
                        break;
                    default:
                        /* Передать обработку прочих клавиш установленному обработчику */
                        if(hotkeysHandler()) {
                            hotkeysHandler()->handleHotkey(key_evt->wVirtualKeyCode,
                                                           selectedAction(), this);
                            /* Так как обработчики горячих клавиш могут изменять структуру
                             * меню или отображать информационные блоки, необходимо очистить
                             * консоль после их работы */
                            clearConsole();
                            _repaint();
                        }
                    }
                }
                break;
            case MOUSE_EVENT:
                mouse_evt = &next_record.Event.MouseEvent;
#ifdef DEBUG
            {
                FILE* f_debug_log = fopen("debug.log", "a");
                fputs("Mouse event!\n", f_debug_log);
                fclose(f_debug_log);
            }
#endif
                break;
            case WINDOW_BUFFER_SIZE_EVENT:
#ifdef DEBUG
            {
                FILE* f_debug_log = fopen("debug.log", "a");
                fputs("Window buffer size event!\n", f_debug_log);
                fclose(f_debug_log);
            }
#endif
//                win_evt = &next_record.Event.WindowBufferSizeEvent;
//                _resize(win_evt->dwSize);
//                _repaint();
                break;
            }
        }
	}
}

//-----------------------------------------------------------------------------
void IMenu::clearConsole() {
	system("cls");
}

//-----------------------------------------------------------------------------
int IMenu::actionsCount() const {
	return _actions.size();
}

//-----------------------------------------------------------------------------
void IMenu::selectNextMenuAction() {
    if( _selected_action_idx < (actionsCount()-1) ) {
        ++_selected_action_idx;
    }
    else {
        _selected_action_idx = actionsCount()-1;
    }

    if( (_topmost_visible_action_idx + _visible_actions_count-1) < _selected_action_idx ) {
        ++_topmost_visible_action_idx;
    }
}

//-----------------------------------------------------------------------------
void IMenu::selectPreviousMenuAction() {
    if(0 < _selected_action_idx) {
        --_selected_action_idx;
    }

    if(_selected_action_idx < _topmost_visible_action_idx) {
        --_topmost_visible_action_idx;
    }
}

//-----------------------------------------------------------------------------
void IMenu::addAction(IMenuAction *action) {
    _actions.push_back(action);

    /* Если до этого меню не содержало пунктов, выбрать первый пункт меню */
    if(NO_SELECTION == _selected_action_idx) {
        _selected_action_idx = 0;
    }
}

//-----------------------------------------------------------------------------
void IMenu::_printMenuActions() const {
    std::vector<IMenuAction*>::const_iterator iter = _actions.begin();

    /* Переместить итератор к пункту меню, который в данный момент
     * должен отображаться в самом верху экрана */
    for(int i = 0; i < _topmost_visible_action_idx; ++i) {
        ++iter;
    }

    /* Переместить курсор в левый верхний угол */
    moveConsoleCursorInTopLeftCorner();

    /* Верхний колонтитул */
    std::string menu_header = "'" + name() + "' || " + header() + "  ";
    /* Если за верхней границей экрана есть скрытые пункты меню,
     * вывести информацию о них в заголовке меню */
    if(_isHiddenMenuEntriesBeforeTop()) {
        std::stringstream ss_menu_header;
        ss_menu_header
                << "|  < "
                << _hiddenMenuEntriesBeforeTopCount()
                << " items above... > ";
        menu_header += ss_menu_header.str();
    }
        std::cout
                << std::left
                << std::setfill('/')
                << std::setw(79)
                << menu_header
                << std::endl;

    /* Вывести пункты меню */
    int action_idx = 0;
    for(; iter != _actions.end() && action_idx < _visible_actions_count;
        ++iter) {
        IMenuAction* next_action = *iter;

        /* Напечатать выбранный элемент меню отлично от остальных */
        if(_selected_action_idx == (_topmost_visible_action_idx + action_idx)) {
            HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

            /* Сохранить предыдущие атрибуты текста в консоли */
            CONSOLE_SCREEN_BUFFER_INFO console_info;
            GetConsoleScreenBufferInfo(hConsole, &console_info);

            /* Установить особый цвет текста и фона для выделенного элемента */
            SetConsoleTextAttribute(hConsole, getConsoleTextAttributesForSelectedItem());

            /* Вывести текст пункта меню */
            std::cout
                    << std::left
                    << std::setfill(' ')
                    << std::setw(79)
                    << next_action->text()
                    << std::endl;

            /* Восстановить предыдущие атрибуты текста в консоли */
            SetConsoleTextAttribute(hConsole, console_info.wAttributes);
        }
        else {
            /* Вывести текст пункта меню обычным шрифтом */
            std::cout
                    << std::left
                    << std::setfill(' ')
                    << std::setw(79)
                    << next_action->text()
                    << std::endl;
        }
        ++action_idx;
    }

    /* Нижний колонтитул */
    std::string menu_footer = footer();
    if(_isHiddenMenuEntriesAfterBottom()) {
        std::stringstream ss_menu_footer;
        ss_menu_footer
                << " < "
                << _hiddenMenuEntriesAfterBottomCount()
                << " items more... > ";
        menu_footer += ss_menu_footer.str();
    }
    std::cout
            << std::left
            << std::setfill('/')
            << std::setw(79)
            << menu_footer;
}

//-----------------------------------------------------------------------------
void IMenu::_repaint() {
    /* Очистить консоль */
//    _clearScreen();

    /* Вывести пункты меню */
    _printMenuActions();
}

//-----------------------------------------------------------------------------
CONSOLE_FONT_INFOEX IMenu::_getConsoleFontForSelectedAction() {
    CONSOLE_FONT_INFOEX font_info;
    font_info.cbSize = sizeof(CONSOLE_FONT_INFOEX);
    font_info.dwFontSize.X = 20;
    font_info.dwFontSize.Y = 50;
    font_info.FontWeight = 700;
    font_info.nFont = 1;
    font_info.FontFamily = FF_DECORATIVE;
    lstrcpynW(font_info.FaceName, L"Courier", LF_FACESIZE);

    return font_info;
}

//-----------------------------------------------------------------------------
IMenuAction* IMenu::actionAt(int idx) const {
    if(0 <= idx
            && idx < actionsCount()) {
        return _actions.at(idx);
    }

    return NULL;
}

//-----------------------------------------------------------------------------
void IMenu::clear() {
    removeAllActions();
}

//-----------------------------------------------------------------------------
void IMenu::_resize(COORD size) {
    int screen_rows_count = size.Y / _line_height;
    /* Две строки у верхнего и нижнего края окна резервируются для
     * информирования пользователя, есть ли за ними скрытые пункты
     * меню */
    _visible_actions_count = screen_rows_count - 2;
    if(_visible_actions_count < 0) {
        _visible_actions_count = 0;
    }

    _menu_page_size = _visible_actions_count - 1;
}

//-----------------------------------------------------------------------------
void IMenu::_gotoNextMenuPage() {
    for(int i = 0; i < _menu_page_size; ++i) {
        selectNextMenuAction();
    }
}

//-----------------------------------------------------------------------------
void IMenu::_gotoPreviousMenuPage() {
    for(int i = 0; i < _menu_page_size; ++i) {
        selectPreviousMenuAction();
    }
}

//-----------------------------------------------------------------------------
std::string IMenu::name() const {
    return _name;
}

//-----------------------------------------------------------------------------
void IMenu::setName(std::string name) {
    _name = name;
}

//-----------------------------------------------------------------------------
bool IMenu::_isHiddenMenuEntriesBeforeTop() const {
    return (0 < _topmost_visible_action_idx);
}

//-----------------------------------------------------------------------------
bool IMenu::_isHiddenMenuEntriesAfterBottom() const {
    /* Индекс пункта меню, который в данный момент отображается последним */
    int last_visible_entry_idx = _topmost_visible_action_idx
            + _visible_actions_count;

    return (last_visible_entry_idx < actionsCount()-1);
}

//-----------------------------------------------------------------------------
int IMenu::_hiddenMenuEntriesBeforeTopCount() const {
    return _topmost_visible_action_idx;
}

//-----------------------------------------------------------------------------
int IMenu::_hiddenMenuEntriesAfterBottomCount() const {
    int last_visible_entry_idx = _topmost_visible_action_idx
            + _visible_actions_count;

    int hidden_items_count = actionsCount()-1 - last_visible_entry_idx;
    return hidden_items_count;
}

//-----------------------------------------------------------------------------
std::string IMenu::header() const {
    return _header;
}

//-----------------------------------------------------------------------------
void IMenu::setHeader(std::string header) {
    _header  = header;
}

//-----------------------------------------------------------------------------
std::string IMenu::footer() const {
    return _footer;
}

//-----------------------------------------------------------------------------
void IMenu::setFooter(std::string footer) {
    _footer = footer;
}

//-----------------------------------------------------------------------------
void IMenu::removeAction(IMenuAction *action) {
    if(action) {
        std::vector<IMenuAction*>::iterator iter_lookup =
                std::find(_actions.begin(), _actions.end(), action);

        if(iter_lookup != _actions.end()) {
            /* Уничтожить пункт меню */
            IMenuAction* action_to_remove = *iter_lookup;
            delete action_to_remove;
            _actions.erase(iter_lookup);

            /* Выправить индексы меню */
            if(0 < _topmost_visible_action_idx
                    && actionsCount() <= _topmost_visible_action_idx) {
                --_topmost_visible_action_idx;
            }
            if(0 < _selected_action_idx
                    && actionsCount() <= _selected_action_idx) {
                --_selected_action_idx;
            }
        }
    }
}

//-----------------------------------------------------------------------------
void IMenu::removeAction(int index) {
    bool is_index_in_correct_range = (0 <= index && index < actionsCount());

    if(is_index_in_correct_range) {
        IMenuAction* action = actionAt(index);
        removeAction(action);
    }
}

//-----------------------------------------------------------------------------
void IMenu::removeAllActions() {
    int actions_count = actionsCount();
    for(int i = 0; i < actions_count; ++i) {
        removeAction(0);
    }

    /* Сбросить индекс текущего пункта меню */
    _selected_action_idx = 0;

    /* Сбросить индекс верхнего видимого пункта меню */
    _topmost_visible_action_idx = 0;
}

//-----------------------------------------------------------------------------
IMenuHotkeysHandler* IMenu::hotkeysHandler() const {
    return _hotkeys_handler;
}

//-----------------------------------------------------------------------------
void IMenu::setHotkeysHandler(IMenuHotkeysHandler *handler) {
    _hotkeys_handler = handler;
}

//-----------------------------------------------------------------------------
IMenuAction* IMenu::selectedAction() const {
    return actionAt(selectedIndex());
}

//-----------------------------------------------------------------------------
void IMenu::moveConsoleCursorInTopLeftCorner() {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    COORD pos;
    pos.X = 0;
    pos.Y = 0;
    SetConsoleCursorPosition(hConsole, pos);
}

//-----------------------------------------------------------------------------
int IMenu::indexOfAction(IMenuAction *action) const {
    int index = -1;
    int i = 0;
    std::vector<IMenuAction*>::const_iterator iter = _actions.begin();
    for(; iter != _actions.end(); ++iter) {
        IMenuAction* next_action = *iter;
        if(action == next_action) {
            index = i;
            break;
        }
        ++i;
    }

    return index;
}

//-----------------------------------------------------------------------------
int IMenu::selectedIndex() const {
    return _selected_action_idx;
}

//-----------------------------------------------------------------------------
void IMenu::setSelectedAction(int index) {
    if(0 <= index && index < actionsCount()) {
        _selected_action_idx = index;
    }
}

//-----------------------------------------------------------------------------
bool IMenu::isClosedByUser() const {
    return _is_closed_by_user;
}

//-----------------------------------------------------------------------------
void IMenu::setClosedByUser(bool state) {
    _is_closed_by_user = state;
}

//-----------------------------------------------------------------------------
std::string IMenu::defaultMenuHeader() const {
    return "";
}

//-----------------------------------------------------------------------------
std::string IMenu::defaultMenuFooter() const {
    return "Press F1 for help";
}

//-----------------------------------------------------------------------------
void IMenu::showMenuHelp() const {
    if(_isHelpTextProvided()) {
        clearConsole();
        moveConsoleCursorInTopLeftCorner();
        std::cout << menuHelpText();
        system("pause");
    }
}

//-----------------------------------------------------------------------------
bool IMenu::_isHelpTextProvided() const {
    return (!menuHelpText().empty());
}

//-----------------------------------------------------------------------------
std::string IMenu::menuHelpText() const {
    return "";
}

//-----------------------------------------------------------------------------
int IMenu::getConsoleTextAttributesForMenu() {
    return FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED;
}

//-----------------------------------------------------------------------------
int IMenu::getConsoleTextAttributesForSelectedItem() {
    return BACKGROUND_GREEN | BACKGROUND_BLUE | BACKGROUND_RED;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
