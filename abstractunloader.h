#ifndef ABSTRACTUNLOADER_H
#define ABSTRACTUNLOADER_H

#include <unstructuredgrid.h>

class AbstractUnloader
{
public:
    virtual void unload(const std::string& path, UnstructuredGrid& grid) const = 0;
};

#endif // ABSTRACTUNLOADER_H
