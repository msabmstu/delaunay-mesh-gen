#ifndef ABSTRACTLOADER_H
#define ABSTRACTLOADER_H

#include <unstructuredgrid.h>

class AbstractLoader
{
public:
    virtual void load(const std::string& path, UnstructuredGrid& grid) const = 0;
};

#endif // ABSTRACTLOADER_H
