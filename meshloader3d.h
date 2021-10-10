#ifndef MESHLOADER3D_H
#define MESHLOADER3D_H

#include <abstractloader.h>

class MeshLoader3D : public AbstractLoader
{
public:
    virtual void load(const std::string& path, UnstructuredGrid& grid) const override;
};

#endif // MESHLOADER3D_H
