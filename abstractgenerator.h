#ifndef ABSTRACTGENERATOR_H
#define ABSTRACTGENERATOR_H

#include <unstructuredgrid.h>
#include <unstructuredgrid.h>
#include <memory>
#include <functional>

class AbstractGenerator
{
public:
    virtual void generate() = 0;
    virtual void initialize(std::shared_ptr<UnstructuredGrid> grid_ptr, bool counter_clockwise = true, std::function<double(const BaseNode&)> rho = 0)= 0;
    virtual ~AbstractGenerator() {}
};

#endif // ABSTRACTGENERATOR_H
