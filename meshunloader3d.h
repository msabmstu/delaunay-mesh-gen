#ifndef MESHUNLOADER3D_H
#define MESHUNLOADER3D_H

#include <abstractunloader.h>
#include <iomanip>

class MeshUnloader3D : public AbstractUnloader
{
public:
    virtual void unload(const std::string& path, UnstructuredGrid& grid) const override;
};


#endif // MESHUNLOADER3D_H
