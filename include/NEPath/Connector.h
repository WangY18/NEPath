#pragma once
#include <NEPath/path.h>
#include <functional>

namespace nepath
{
    class Connector
    {
    public:
        using NodeDeletionObserver = std::function<void(pathnode *)>;

        static path ConnectedFermatSpiral_MultMinimum(pathnode *root, double dis, double in = -1.0); // Connected Fermat Spiral (CFS)
        static path ConnectedFermatSpiral_MultMinimum(pathnode *root, double dis, const NodeDeletionObserver &before_node_delete,
                                                      double in = -1.0); // CFS with explicit node-lifetime observation
        static path ConnectedDFS(pathnode *root);                                                    // Depth First Search
    private:
        static path FermatSpiral_SingleMinimum(pathnode *root, double dis, double in = -1.0); // CFS with a single local minimum
    };
}
