#pragma once
#include "path.h"

namespace nepath
{
    class Connector
    {
    public:
        static path ConnectedFermatSpiral_MultMinimum(pathnode *root, double dis, double in = -1.0); // Connected Fermat Spiral (CFS)
        static path ConnectedDFS(pathnode *root);                                                    // Depth First Search
    private:
        static path FermatSpiral_SingleMinimum(pathnode *root, double dis, double in = -1.0); // CFS with a single local minimum
    };
}