#ifndef IMENUHOTKEYHANDLER_H
#define IMENUHOTKEYHANDLER_H

#include <string>
#include <sstream>
#include <iomanip>

class IMenu;
class IMenuAction;

class IMenuHotkeysHandler
{
public:
    IMenuHotkeysHandler();
    virtual ~IMenuHotkeysHandler();

    virtual void handleHotkey(int key_code, IMenuAction* action, IMenu* menu) = 0;

    static std::string formatHotkeyInfoString(std::string key,
                                              std::string description,
                                              int line_width);

protected:
};

#endif // IMENUHOTKEYHANDLER_H
