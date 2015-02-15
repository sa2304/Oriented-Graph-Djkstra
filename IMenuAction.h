#pragma once

#include <string>

class IMenuAction
{
public:
	IMenuAction(void);
	virtual ~IMenuAction(void);

    virtual std::string text() const = 0;
};

